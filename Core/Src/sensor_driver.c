/**
  ******************************************************************************
  * @file    i_nucleo_lrwan1_humidity.c
  * @author  MEMS Application Team
  * @brief   This file provides a set of functions needed to manage the humidity sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>

#include "i2c.h"
#include "flat_top_win.h"
#include "arm_math.h"

#include "sensor_board.h"
#include "sensor_hts221_board.h"
#include "sensor_lps22hb_board.h"
#include "sensor_lpm303agr_board.h"

#include "sensor_driver.h"
#include "sensor_hts221_driver.h"
#include "sensor_lps22hb_driver.h"
#include "sensor_lpm303agr_driver.h"

DrvContextTypeDef HUMIDITY_SensorHandle[ HUMIDITY_SENSORS_MAX_NUM ];
HUMIDITY_Data_t HUMIDITY_Data[ HUMIDITY_SENSORS_MAX_NUM ]; /* Humidity - all. */

DrvContextTypeDef TEMPERATURE_SensorHandle[ TEMPERATURE_SENSORS_MAX_NUM ];
TEMPERATURE_Data_t TEMPERATURE_Data[ TEMPERATURE_SENSORS_MAX_NUM ]; /* Temperature - all. */

DrvContextTypeDef PRESSURE_SensorHandle[ PRESSURE_SENSORS_MAX_NUM ];
PRESSURE_Data_t PRESSURE_Data[ PRESSURE_SENSORS_MAX_NUM ]; /* Pressure - all. */

DrvContextTypeDef MAGNETO_SensorHandle[ MAGNETO_SENSORS_MAX_NUM ];
MAGNETO_Data_t MAGNETO_Data[ MAGNETO_SENSORS_MAX_NUM ]; // Magnetometer - all.

DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.

uint16_t pressure = 0;
int16_t temperature = 0;
uint16_t humidity = 0;
uint16_t magneto = 0;
int16_t accelero[3] = {0, 0, 0};
uint32_t fftMaxAmpl = 0;
uint32_t fftMaxFreq = 0;

void *hHUMIDITY = NULL;
void *hTEMPERATURE = NULL;
void *hPRESSURE = NULL;
void *hMAGNETO = NULL;
void *hACCELERO = NULL;

/**
 * @brief Initialize a humidity sensor
 * @param id the humidity sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Init(HUMIDITY_ID_t id, void **handle)
{

  *handle = NULL;

  switch (id)
  {
    case HUMIDITY_SENSORS_AUTO:
    default:
    {
      if (BSP_HTS221_HUMIDITY_Init(handle)  == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case HTS221_H_0:
    {
      if (BSP_HTS221_HUMIDITY_Init(handle)  == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize a humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_DeInit(void **handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->DeInit == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->DeInit(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Enable humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Enable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Enable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Disable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Disable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Disable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the humidity sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_IsInitialized(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the humidity sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_IsEnabled(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the humidity sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_IsCombo(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the humidity sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Get_Instance(void *handle, uint8_t *instance)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (instance == NULL)
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the humidity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Get_WhoAmI(void *handle, uint8_t *who_am_i)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (who_am_i == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI(ctx, who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Check_WhoAmI(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Check_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Check_WhoAmI(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the humidity value
 * @param handle the device handle
 * @param humidity pointer where the value is written [%]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Get_Hum(void *handle, float *humidity)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (humidity == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Hum == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Hum(ctx, humidity) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the humidity sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Get_ODR(void *handle, float *odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (odr == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the humidity sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR(void *handle, SensorOdr_t odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the humidity sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR_Value(void *handle, float odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR_Value == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR_Value(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Read_Reg(void *handle, uint8_t reg, uint8_t *data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Write_Reg(void *handle, uint8_t reg, uint8_t data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Write_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Write_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get humidity data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HUMIDITY_Get_DRDY_Status(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  HUMIDITY_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (HUMIDITY_Drv_t *)ctx->pVTable;

  if (driver->Get_DRDY_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_DRDY_Status(ctx, status) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize a temperature sensor
 * @param id the temperature sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Init(TEMPERATURE_ID_t id, void **handle)
{

  *handle = NULL;

  switch (id)
  {
    case TEMPERATURE_SENSORS_AUTO:
    default:
    {
      /* Try to init the LPS22HB DIL24 first */
      if (BSP_LPS22HB_TEMPERATURE_Init(handle) == COMPONENT_ERROR)
      {
	    /* Try to init the HTS221 on board if we do not use the LPS22HB DIL24 and LPS25H/B DIL24 */
	    if (BSP_HTS221_TEMPERATURE_Init(handle) == COMPONENT_ERROR)
	    {
		  return COMPONENT_ERROR;
	    }
      }
      break;
    }
    case HTS221_T_0:
    {
      if (BSP_HTS221_TEMPERATURE_Init(handle) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }

    case LPS22HB_T_0:
    {
      if (BSP_LPS22HB_TEMPERATURE_Init(handle) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize a temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_DeInit(void **handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->DeInit == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->DeInit(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}

/**
 * @brief Enable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Enable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Enable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Disable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Disable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Disable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check if the temperature sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsInitialized(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}

/**
 * @brief Check if the temperature sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsEnabled(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}

/**
 * @brief Check if the temperature sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsCombo(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Instance(void *handle, uint8_t *instance)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (instance == NULL)
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_WhoAmI(void *handle, uint8_t *who_am_i)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (who_am_i == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI(ctx, who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Check_WhoAmI(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Check_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Check_WhoAmI(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value
 * @param handle the device handle
 * @param temperature pointer where the value is written [C]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Temp(void *handle, float *temperature)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (temperature == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Temp == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Temp(ctx, temperature) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_ODR(void *handle, float *odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (odr == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR(void *handle, SensorOdr_t odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR_Value(void *handle, float odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR_Value == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR_Value(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Read_Reg(void *handle, uint8_t reg, uint8_t *data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Write_Reg(void *handle, uint8_t reg, uint8_t data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Write_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Write_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get temperature data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_DRDY_Status(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (TEMPERATURE_Drv_t *)ctx->pVTable;

  if (driver->Get_DRDY_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_DRDY_Status(ctx, status) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get FIFO THR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO THR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Fth_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Fth_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO FULL status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO FULL status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Full_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO OVR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO OVR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Ovr_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Ovr_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO data (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *pressure pointer to FIFO pressure data
 * @param *temperature pointer to FIFO temperature data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Data_Ext(void *handle, float *pressure, float *temperature)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (pressure == NULL || temperature == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Data == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data(ctx, pressure, temperature);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get number of unread FIFO samples (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint8_t *nSamples)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (nSamples == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Num_Of_Samples == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Num_Of_Samples(ctx, nSamples);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO mode (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Mode_Ext(void *handle, uint8_t mode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Mode == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Mode(ctx, mode);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext(void *handle, uint8_t interrupt)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Interrupt == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Interrupt(ctx, interrupt);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext(void *handle, uint8_t interrupt)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Reset_Interrupt == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Reset_Interrupt(ctx, interrupt);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO watermark (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param watermark FIFO watermark
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext(void *handle, uint8_t watermark)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Watermark_Level == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level(ctx, watermark);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO to stop on FTH (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable stopping on FTH interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Stop_On_Fth == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Stop_On_Fth(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief FIFO usage (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable FIFO
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Usage_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_T_ExtDrv_t *extDriver = (LPS22HB_T_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Usage == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Usage(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Initialize a pressure sensor
 * @param id the pressure sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Init(PRESSURE_ID_t id, void **handle)
{
  *handle = NULL;

  switch (id)
  {
    case PRESSURE_SENSORS_AUTO:
    default:
    {
      /* Try to init the LPS22HB DIL24 first */
      if (BSP_LPS22HB_PRESSURE_Init(handle) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LPS22HB_P_0:
    {
      if (BSP_LPS22HB_PRESSURE_Init(handle) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize a pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_DeInit(void **handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->DeInit == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->DeInit(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}

/**
 * @brief Enable pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Sensor_Enable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Enable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Sensor_Disable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Disable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Disable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check if the pressure sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_IsInitialized(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}

/**
 * @brief Check if the pressure sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_IsEnabled(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}

/**
 * @brief Check if the pressure sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_IsCombo(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}

/**
 * @brief Get the pressure sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Get_Instance(void *handle, uint8_t *instance)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (instance == NULL)
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the pressure sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Get_WhoAmI(void *handle, uint8_t *who_am_i)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (who_am_i == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_WhoAmI(ctx, who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Check_WhoAmI(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Check_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Check_WhoAmI(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the pressure value
 * @param handle the device handle
 * @param pressure pointer where the value is written [hPa]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Get_Press(void *handle, float *pressure)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (pressure == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Press == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_Press(ctx, pressure) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the pressure sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Get_ODR(void *handle, float *odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (odr == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Get_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the pressure sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Set_ODR(void *handle, SensorOdr_t odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the pressure sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Set_ODR_Value(void *handle, float odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR_Value == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR_Value(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}
/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Read_Reg(void *handle, uint8_t reg, uint8_t *data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Write_Reg(void *handle, uint8_t reg, uint8_t data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Write_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Write_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get pressure data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_Get_DRDY_Status(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  PRESSURE_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (PRESSURE_Drv_t *)ctx->pVTable;

  if (driver->Get_DRDY_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_DRDY_Status(ctx, status) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get FIFO THR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO THR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Fth_Status_Ext(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Fth_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Fth_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO FULL status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO FULL status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Full_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Full_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO OVR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO OVR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Ovr_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Ovr_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Ovr_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get FIFO data (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *pressure pointer to FIFO pressure data
 * @param *temperature pointer to FIFO temperature data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Data_Ext(void *handle, float *pressure, float *temperature)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (pressure == NULL || temperature == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Data == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data(ctx, pressure, temperature);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Get number of unread FIFO samples (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint8_t *nSamples)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (nSamples == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Num_Of_Samples == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Num_Of_Samples(ctx, nSamples);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO mode (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Mode_Ext(void *handle, uint8_t mode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Mode == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Mode(ctx, mode);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Interrupt_Ext(void *handle, uint8_t interrupt)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Interrupt == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Interrupt(ctx, interrupt);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Reset_Interrupt_Ext(void *handle, uint8_t interrupt)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Reset_Interrupt == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Reset_Interrupt(ctx, interrupt);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO watermark (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param watermark FIFO watermark
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Watermark_Level_Ext(void *handle, uint8_t watermark)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Watermark_Level == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level(ctx, watermark);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief Set FIFO to stop on FTH (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable stopping on FTH interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Stop_On_Fth_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Stop_On_Fth == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Stop_On_Fth(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief FIFO usage (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable FIFO
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PRESSURE_FIFO_Usage_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if (ctx->who_am_i == LPS22HB_WHO_AM_I_VAL)
  {
    LPS22HB_P_ExtDrv_t *extDriver = (LPS22HB_P_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Usage == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Usage(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}


DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle )
{

  *handle = NULL;

  switch(id)
  {
    case MAGNETO_SENSORS_AUTO:
    default:
    {
      if( BSP_LSM303AGR_MAGNETO_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LSM303AGR_M_0:
    {
      if( BSP_LSM303AGR_MAGNETO_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize a magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}


/**
 * @brief Enable magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the magnetometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the magnetometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the magnetometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the magnetometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written [mgauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( magnetic_field == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes( ctx, magnetic_field ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( value == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}




/**
 * @brief Get the magnetometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( odr == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( fullScale == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get magnetometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( MAGNETO_Drv_t * )ctx->pVTable;

  if ( driver->Get_DRDY_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_DRDY_Status( ctx, status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize an accelerometer sensor
 * @param id the accelerometer sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Init( ACCELERO_ID_t id, void **handle )
{

  *handle = NULL;

  switch(id)
  {
    case ACCELERO_SENSORS_AUTO:
    case LSM303AGR_X_0:
    {
      if( BSP_LSM303AGR_ACCELERO_Init(handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_DeInit( void **handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}


/**
 * @brief Enable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written [mg]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes( void *handle, SensorAxes_t *acceleration )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(acceleration == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes( ctx, acceleration ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(value == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw( ctx, value) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [mg/LSB]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Sensitivity( void *handle, float *sensitivity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(sensitivity == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(odr == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_FS( void *handle, float *fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(fullScale == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_FS == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS( void *handle, SensorFs_t fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS_Value( void *handle, float fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(xyz_enabled == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes_Status( ctx, xyz_enabled ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Axes_Status( void *handle, uint8_t *enable_xyz )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(enable_xyz == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Set_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Set_Axes_Status( ctx, enable_xyz ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get accelerometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Get_DRDY_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_DRDY_Status( ctx, status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Clear the accelerometer sensor DRDY
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_ClearDRDY_Ext( void *handle, ACTIVE_AXIS_t axisActive )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }  
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable; 
      
      if ( extDriver->ClearDRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->ClearDRDY( ctx, axisActive ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    } 
  }
  
  return COMPONENT_ERROR;
}

/**
 * @brief Set the accelerometer sensor DRDY on INT1
 * @param handle the device handle
 * @param drdyStatus enable/disable DRDY on INT1 value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_INT1_DRDY_Ext( void *handle, INT1_DRDY_CONFIG_t drdyStatus )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Set_INT1_DRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_INT1_DRDY( ctx, drdyStatus ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;  
    }
  }
  
  return COMPONENT_ERROR;
}


/**
 * @brief Get the accelerometer sensor super raw data from one axis
 * @param handle the device handle
 * @param acceleration pointer where the super raw value of the axis is written
 * @param axis axis to be read
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_SuperRawAxes_Ext( void *handle, int16_t *acceleration, ACTIVE_AXIS_t axis )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;  
      
      if ( extDriver->Get_AxesSuperRaw == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Get_AxesSuperRaw( ctx, acceleration, axis ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    }  
  } 
      
  return COMPONENT_ERROR;
}


/**
 * @brief Get the accelerometer sensor operating mode
 * @param handle the device handle
 * @param opMode pointer where the operating mode value is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_OpMode_Ext( void *handle, OP_MODE_t *opMode )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;

      if ( extDriver->Get_OpMode == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Get_OpMode( ctx, opMode ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    } 
  } 
    
  return COMPONENT_ERROR; 
}

/**
 * @brief Set the accelerometer sensor operating mode
 * @param handle the device handle
 * @param opMode the value of operating mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_OpMode_Ext( void *handle, OP_MODE_t opMode )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable; 
      
      if ( extDriver->Set_OpMode == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_OpMode( ctx, opMode ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;  
    }   
  } 
  
  return COMPONENT_ERROR;
}

/**
 * @brief Set the accelerometer sensor active axis
 * @param handle the device handle
 * @param axis pointer where the active axis value is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Active_Axis_Ext( void *handle, ACTIVE_AXIS_t *axis )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Get_Active_Axis == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Get_Active_Axis( ctx, axis ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } 
  
  return COMPONENT_ERROR;
}


/**
 * @brief Set the accelerometer sensor active axis
 * @param handle the device handle
 * @param axis the value of active axis to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Active_Axis_Ext( void *handle, ACTIVE_AXIS_t axis )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;  
      
      if ( extDriver->Set_Active_Axis == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_Active_Axis( ctx, axis ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;
    }
  } 
  
  return COMPONENT_ERROR;
}

/**
 * @brief Enable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @param mode the value of HP Filter mode to be set
 * @param cutoff the value of HP Filter cutoff to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_HP_Filter_Ext( void *handle )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable; 
      
      if ( extDriver->Enable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Enable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    }
  }
  
  return COMPONENT_ERROR;
}



/**
 * @brief Disable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_HP_Filter_Ext( void *handle )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Disable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Disable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    } 
  } 
  
  return COMPONENT_ERROR;
}


typedef struct
{
  uint16_t       magnitude;
  uint16_t       samples;
  uint8_t        hp_filter;
  uint8_t        switch_HP_to_DC_null;

} fft_settings_t;

///**
// * @brief 32-bit floating-point type definition.
// */
//typedef float float32_t;
//
///**
// * @brief 64-bit floating-point type definition.
// */
//typedef double float64_t;

typedef struct
{
  float32_t      bin_max_value;
  uint32_t       bin_max_index;
  float32_t      ftw_max_value;
  uint32_t       ftw_max_index;
  float32_t      max_value;
  uint32_t       max_index;
} fft_out_data_t;


const float lsm303agr_odr[] = {5, 200, 400, 1344, 1620, 5376};
const float lsm303agr_odr_om[] = {12, 200, 400, 1620, 5376, 0, 200, 400, 1344, 0, 200, 400, 1344};
const uint32_t lsm303agr_fs[] = {4, 2, 4, 8, 16};
const OP_MODE_t lsm303agr_opmode[] = {LOW_PWR_MODE, NORMAL_MODE, HIGH_RES_MODE};
const ACTIVE_AXIS_t lsm303agr_axis[] = {X_AXIS, Y_AXIS, Z_AXIS, ALL_ACTIVE };
const uint32_t lsm303agr_samples_list[] = {3, 256, 512, 1024};

fft_out_data_t   out_data = {.max_index = 0, .max_value = 0, .bin_max_index = 0, .ftw_max_index = 0, .ftw_max_value = 0};
fft_settings_t   settings = {.magnitude = 1000, .hp_filter = 0, .switch_HP_to_DC_null = 0, .samples = 512 };
float            odr_measured = 0; /* Calculated value of ODR (for ODR calibration) */
uint16_t         freq_axis[50];

#define MAX_SAMPLES             1024

float32_t        fft_input[MAX_SAMPLES];
float32_t        fft_output[MAX_SAMPLES];
float32_t        fft_text_plot_data[51];
float32_t        fft_tmp[MAX_SAMPLES];
float32_t        fft_tmp2[MAX_SAMPLES];
int16_t          tmp_array[MAX_SAMPLES];

arm_rfft_fast_instance_f32 real_fft_Instance;

volatile uint8_t AXL_INT_received = 0;
volatile uint8_t fft_data_rdy = 0;


/**
 * @brief  Reset interrupts
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t reset_INT( void )
{
	ACTIVE_AXIS_t axis_active;

	/* Get Sensing axis */
	if(BSP_ACCELERO_Get_Active_Axis_Ext( hACCELERO, &axis_active ) != COMPONENT_OK)
	{
	  return 1;
	}
	
	/* Clear DRDY */
	if(BSP_ACCELERO_ClearDRDY_Ext( hACCELERO, axis_active ) != COMPONENT_OK)
	{
	  return 1;
	}
	
	AXL_INT_received = 0;
	 // printf("8. AXL_INT_received: %d\n\r", AXL_INT_received);
  	return 0;
}

/**
 * @brief  Enable DRDY
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t enable_DRDY( void )
{
  ACTIVE_AXIS_t axis_active;
  AXL_INT_received = 0;
      //printf("6. AXL_INT_received: %d\n\r", AXL_INT_received);
  /* Get Sensing axis */
  if(BSP_ACCELERO_Get_Active_Axis_Ext( hACCELERO, &axis_active ) != COMPONENT_OK)
  {
    return 1;
  }
  /* Enable DRDY */
  if(BSP_ACCELERO_Set_INT1_DRDY_Ext( hACCELERO, INT1_DRDY_ENABLED ) != COMPONENT_OK)
  {
    return 1;
  }
  /* Clear DRDY */
  if(BSP_ACCELERO_ClearDRDY_Ext( hACCELERO, axis_active ) != COMPONENT_OK)
  {
    return 1;
  }

  return 0;
}


/**
 * @brief  Disable DRDY
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t disable_DRDY( void )
{
  /* Disable DRDY */
  if(BSP_ACCELERO_Set_INT1_DRDY_Ext( hACCELERO, INT1_DRDY_DISABLED ) != COMPONENT_OK)
  {
    return 1;
  }

  return 0;
}


#if 0
/**
 * @brief  Initialize FFT demo
 *
 * @param  func Program functionality selector
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t init_fft()
{

  if(BSP_ACCELERO_Disable_HP_Filter_Ext( hACCELERO ) != COMPONENT_OK)
  {
	return 1;
  }

  if(BSP_ACCELERO_Set_Active_Axis_Ext( hACCELERO, Z_AXIS ) != COMPONENT_OK)
  {
    return 1;
  }
  /* Set ODR to 400Hz for FFT demo */
  if(BSP_ACCELERO_Set_ODR_Value( hACCELERO, 400.0f ) != COMPONENT_OK)
  {
    return 1;
  }

  /* Turn-on time delay */
  HAL_Delay(40);

  if( BSP_ACCELERO_Set_OpMode_Ext( hACCELERO, NORMAL_MODE ) != COMPONENT_OK )
  {
    return 1;
  }

  if(enable_DRDY())
  {
    return 1;
  }

  settings.switch_HP_to_DC_null = 0;
  settings.samples = 512;
  daq_enable = 1;
  table_displayed = 1;
  fft_data_rdy = 0;
  settings.hp_filter = 0;
  init_menu = 1;

  arm_rfft_fast_init_f32(&real_fft_Instance, settings.samples);

  /* Measeure and calculate ODR */
  if(meas_odr())
  {
    return 1;
  }

  /* Calculate values for graph x axis */
  calculate_freq_axis();

   return 0;
}

#endif


/**
 * @brief  Acquire Data from accelerometer
 * @param  void
 * @retval 0 in case of success
 * @retval 1 in case of failure
 * @retval 2 when reset_INT function failed
 * @retval 3 when the time of sampling exceeds the timeout
 * @retval 4 when number of samples in FIFO is not equal to required number of samples
 */
static uint8_t acquire_data( void )
{
  ACTIVE_AXIS_t axis_active;
  float sensitivity = 0.0f;
  int16_t data;
  OP_MODE_t opmode;
  uint16_t amount_of_data = 0;
  uint16_t i;
  uint32_t start = HAL_GetTick();
  uint8_t n_bit_num = 16;
  uint8_t ret_val = 1;

  BSP_ACCELERO_Get_Active_Axis_Ext( hACCELERO, &axis_active );

  BSP_ACCELERO_Get_OpMode_Ext( hACCELERO, &opmode );

  switch(opmode)
  {
  default:
  case LOW_PWR_MODE:
    n_bit_num = 8;
    break;
  case NORMAL_MODE:
    n_bit_num = 10;
    break;
  case HIGH_RES_MODE:
    n_bit_num = 12;
    break;
  }

    BSP_ACCELERO_Get_Sensitivity( hACCELERO, &sensitivity );

    while( amount_of_data != settings.samples )
    {
      if(((HAL_GetTick() - start) > 10000))
      {
        return 3;
      }

      if(AXL_INT_received)
      {
        AXL_INT_received = 0;
        //printf("5. AXL_INT_received: %d\n\r", AXL_INT_received);
        BSP_ACCELERO_Get_SuperRawAxes_Ext( hACCELERO, &data, axis_active );      /* get data */

        tmp_array[amount_of_data] = data;

        amount_of_data++;
      }
    }

    if( amount_of_data == settings.samples )
    {
      for( i=0; i<settings.samples; i++ )
      {
        if( n_bit_num == 8)
        {
          tmp_array[i] >>= 8;

          /* convert the 2's complement 8 bit to 2's complement 16 bit */
          if (tmp_array[i] & 0x0080)
          {
            tmp_array[i] |= 0xFF00;
          }
        } else if( n_bit_num == 10)
        {
          tmp_array[i] >>= 6;

          /* convert the 2's complement 10 bit to 2's complement 16 bit */
          if (tmp_array[i] & 0x0200)
          {
            tmp_array[i] |= 0xFC00;
          }
        } else if( n_bit_num == 12)
        {
          tmp_array[i] >>= 4;

          /* convert the 2's complement 12 bit to 2's complement 16 bit */
          if (tmp_array[i] & 0x0800)
          {
            tmp_array[i] |= 0xF000;
          }
        }

        fft_input[i] = (float)((tmp_array[i] * sensitivity)/1000);
      }

      ret_val = 0;
    }

  return ret_val;
}

/**
 * @brief  Calculate FFT and create data array for FFT graph plot
 * @param  None
 * @retval None
 */
static void calculate_fft( void )
{
  uint32_t ifft_flag = 0;
  uint16_t i, j;

  for(i=0; i<51; i++)
  {
    fft_text_plot_data[i] = 0;
  }

  for(i=0; i<settings.samples; i++)
  {
    fft_tmp[i] = fft_input[i];
  }

  /* Calculate FFT */
  arm_rfft_fast_f32(&real_fft_Instance, fft_tmp, fft_tmp2, ifft_flag);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(fft_tmp2, fft_output, settings.samples);


  /* Flat Top window is supported only when HP filter is on */
  if( !settings.switch_HP_to_DC_null && settings.hp_filter )
  {

    for(i=0; i<settings.samples; i++)
    {
      fft_tmp[i] = fft_input[i]*flat_top_win[i*(1024/settings.samples)];
    }

    /* Calculate FFT using Flat Top window */
    arm_rfft_fast_f32(&real_fft_Instance, fft_tmp, fft_tmp2, ifft_flag);

    /* Process the data through the Complex Magnitude Module for
    calculating the magnitude at each bin */
    arm_cmplx_mag_f32(fft_tmp2, fft_tmp, settings.samples);
  }


  j = 0;

  for(i=0; i<settings.samples/2; i++)
  {
    if( i == 0 )
    {
      if( settings.switch_HP_to_DC_null && settings.hp_filter )
      {
        fft_output[i] = 0; /* if DC nulling enabled */
      } else
      {
        fft_output[i] = (fft_output[i]/(settings.samples))*1000;
        fft_tmp[i] = ((fft_tmp[i]/(settings.samples))*1000)*scale_factor;
      }
    } else
    {
      fft_output[i] = (fft_output[i]/(settings.samples))*2000; /* output*2 */
      fft_tmp[i] = ((fft_tmp[i]/(settings.samples))*2000)*scale_factor;
    }


    if( i == freq_axis[j] )
    {
      j++;
    }

    /* Consider only max value in frequency range of bin */
    if( fft_output[i] > fft_text_plot_data[j] )
    {
      fft_text_plot_data[j] = fft_output[i];
    }
  }

  /* Calculates max_value and returns corresponding BIN value */
  arm_max_f32(fft_output, settings.samples/2, &out_data.max_value, &out_data.max_index);

  /* Calculates max_value and returns corresponding BIN value */
  arm_max_f32(fft_text_plot_data, 51, &out_data.bin_max_value, &out_data.bin_max_index);


  /* Flat Top window is supported only when HP filter is on */
  if( !settings.switch_HP_to_DC_null && settings.hp_filter )
  {
    /* Calculates max_value and returns corresponding BIN value */
    arm_max_f32(fft_tmp, settings.samples/2, &out_data.ftw_max_value, &out_data.ftw_max_index);
  }

  fft_data_rdy = 1;
}

/**
 * @brief  Prepare FFT data for get_fft_data(), get_fft_max_freq() and get_fft_max_freq_amp() functions
 *
 * @param none
 * @retval 0 in case of success
 * @retval 1 in case of failure
 * @retval 2 when reset_INT function failed
 * @retval 3 when the time of sampling exceeds the timeout
 */
uint8_t prepare_fft_data( void )
{
  uint8_t ret_val;
  uint32_t start = HAL_GetTick();

  if(reset_INT())
  {
    return 2;
  }

  do
  {
    if((HAL_GetTick() - start) > 4000)
    {
      return 3;
    }

    ret_val = acquire_data();

  } while(ret_val);

  calculate_fft();
  
  if( !fft_data_rdy )
  {
    return 1;
  }

  return 0;
}

/**
 * @brief  Get amplitude of max frequency using Flat Top window.
 *
 * @param  none
 * @retval calculated max frequency
 */
uint32_t get_fft_max_freq_amp( void )
{
  uint32_t max_val = 0;
  
  /* use calculated max value from Flat Top window when HP filter is on */
  if( !settings.switch_HP_to_DC_null && settings.hp_filter )
  {
    max_val = (uint32_t)out_data.ftw_max_value;
  } else
  {
    max_val = (uint32_t)out_data.max_value;
  }
  
  return max_val;
}

/**
 * @brief  Get max frequency value.
 *
 * @param  none
 * @retval calculated max frequency
 */
uint32_t get_fft_max_freq( void )
{
  return (uint32_t)((out_data.max_index*odr_measured)/(settings.samples));
}

DrvStatusTypeDef SensorDevicesInit()
{
	DrvStatusTypeDef status = COMPONENT_OK;

	status = BSP_HUMIDITY_Init(HTS221_H_0, &hHUMIDITY);
	if (status != COMPONENT_OK) {
		return status;
	}

	status = BSP_TEMPERATURE_Init(HTS221_T_0, &hTEMPERATURE);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_PRESSURE_Init(LPS22HB_P_0, &hPRESSURE);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_MAGNETO_Init(LSM303AGR_M_0, &hMAGNETO);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_ACCELERO_Init(LSM303AGR_X_0, &hACCELERO);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not initialized! \r\n");
	  return status;
	}

	/* Set ODR to 400Hz for FFT */
	status = BSP_ACCELERO_Set_ODR_Value(hACCELERO, 400.0f);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not set ODR value! \r\n");
	  return status;
	}

	/* Enable sensors */
	status = BSP_HUMIDITY_Sensor_Enable(hHUMIDITY);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_TEMPERATURE_Sensor_Enable(hTEMPERATURE);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_PRESSURE_Sensor_Enable(hPRESSURE);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_MAGNETO_Sensor_Enable(hMAGNETO);
	if (status != COMPONENT_OK) {
	  return status;
	}

	status = BSP_ACCELERO_Sensor_Enable(hACCELERO);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not enabled! \r\n");
	  return status;
	}

	return status;
}


DrvStatusTypeDef CollectSensorData() {

	float HUMIDITY_Value = 0;
	float TEMPERATURE_Value = 0;
	float PRESSURE_Value = 0;

	static SensorAxes_t sensValue;
	
	DrvStatusTypeDef status = COMPONENT_OK;

	status = BSP_HUMIDITY_Get_Hum(hHUMIDITY, &HUMIDITY_Value);
	if (status != COMPONENT_OK) {
		printf("Humidity Sensor can not initialized! \r\n");
		return status;
	}

	status = BSP_TEMPERATURE_Get_Temp(hTEMPERATURE, &TEMPERATURE_Value);
	if (status != COMPONENT_OK) {
		printf("Temperature Sensor can not initialized! \r\n");
		return status;
	}

	status = BSP_PRESSURE_Get_Press(hPRESSURE, &PRESSURE_Value);
	if (status != COMPONENT_OK) {
		printf("Pressure Sensor can not initialized! \r\n");
		return status;
	}

	status = BSP_ACCELERO_Set_Active_Axis_Ext(hACCELERO, ALL_ACTIVE);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not set axis to all! \r\n");
		return status;
	}

	HAL_Delay(100);
	/* ACC_Value[0] = X axis, ACC_Value[1] = Y axis, ACC_Value[2] = Z axis */
	BSP_ACCELERO_Get_Axes(hACCELERO, &sensValue);

	status = BSP_ACCELERO_Set_Active_Axis_Ext(hACCELERO, Z_AXIS);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not set axis to Z! \r\n");
		return status;
	}

	if(0 != prepare_fft_data())
    {
        printf("Problem while creating FFT data \r\n");
        return 1;
    }

	humidity    = (uint16_t)(HUMIDITY_Value * 2);            /* in %*2     */
	temperature = (int16_t)(TEMPERATURE_Value * 10);         /* in �C * 10 */
	pressure    = (uint16_t)(PRESSURE_Value * 100 / 10);      /* in hPa / 10 */
	accelero[0] = sensValue.AXIS_X;
	accelero[1] = sensValue.AXIS_Y;
	accelero[2] = sensValue.AXIS_Z;
    fftMaxAmpl = get_fft_max_freq_amp();
    fftMaxFreq = get_fft_max_freq();

	return status;
}

