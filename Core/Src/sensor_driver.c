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
#include "sensor_driver.h"
#include "i2c.h"

static DrvContextTypeDef HUMIDITY_SensorHandle[ HUMIDITY_SENSORS_MAX_NUM ];
static HUMIDITY_Data_t HUMIDITY_Data[ HUMIDITY_SENSORS_MAX_NUM ]; /* Humidity - all. */
static HTS221_H_Data_t HTS221_H_0_Data; /* Humidity - sensor 0. */

static DrvContextTypeDef TEMPERATURE_SensorHandle[ TEMPERATURE_SENSORS_MAX_NUM ];
static TEMPERATURE_Data_t TEMPERATURE_Data[ TEMPERATURE_SENSORS_MAX_NUM ]; /* Temperature - all. */
static HTS221_T_Data_t HTS221_T_0_Data; /* Temperature - sensor 0 HTS221 on board. */
static LPS22HB_T_Data_t LPS22HB_T_0_Data; /* Temperature - sensor 1 LPS22H/B via DIL24. */

static DrvContextTypeDef PRESSURE_SensorHandle[ PRESSURE_SENSORS_MAX_NUM ];
static PRESSURE_Data_t PRESSURE_Data[ PRESSURE_SENSORS_MAX_NUM ]; /* Pressure - all. */
static LPS22HB_P_Data_t LPS22HB_P_0_Data; /* Pressure - sensor 0 LPS22H/B via DIL24. */

uint16_t pressure = 0;
int16_t temperature = 0;
uint16_t humidity = 0;

void *hHUMIDITY = NULL;
void *hTEMPERATURE = NULL;
void *hPRESSURE = NULL;

static DrvStatusTypeDef BSP_HTS221_HUMIDITY_Init(void **handle)
{
  HUMIDITY_Drv_t *driver = NULL;

  if (HUMIDITY_SensorHandle[ HTS221_H_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  HUMIDITY_SensorHandle[ HTS221_H_0 ].who_am_i      = HTS221_WHO_AM_I_VAL;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].address       = HTS221_ADDRESS_DEFAULT;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].instance      = HTS221_H_0;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].isInitialized = 0;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].isEnabled     = 0;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].isCombo       = 1;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].pData         = (void *)&HUMIDITY_Data[ HTS221_H_0 ];
  HUMIDITY_SensorHandle[ HTS221_H_0 ].pVTable       = (void *)&HTS221_H_Drv;
  HUMIDITY_SensorHandle[ HTS221_H_0 ].pExtVTable    = 0;

  HTS221_H_0_Data.comboData = &HTS221_Combo_Data[0];
  HUMIDITY_Data[ HTS221_H_0 ].pComponentData = (void *)&HTS221_H_0_Data;
  HUMIDITY_Data[ HTS221_H_0 ].pExtData       = 0;

  *handle = (void *)&HUMIDITY_SensorHandle[ HTS221_H_0 ];

  driver = (HUMIDITY_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

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
 * @brief Initialize HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_HTS221_TEMPERATURE_Init(void **handle)
{
  TEMPERATURE_Drv_t *driver = NULL;

  if (TEMPERATURE_SensorHandle[ HTS221_T_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].who_am_i      = HTS221_WHO_AM_I_VAL;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].address       = HTS221_ADDRESS_DEFAULT;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].instance      = HTS221_T_0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isInitialized = 0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isEnabled     = 0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isCombo       = 1;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pData         = (void *)&TEMPERATURE_Data[ HTS221_T_0 ];
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pVTable       = (void *)&HTS221_T_Drv;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pExtVTable    = 0;

  HTS221_T_0_Data.comboData = &HTS221_Combo_Data[0];
  TEMPERATURE_Data[ HTS221_T_0 ].pComponentData = (void *)&HTS221_T_0_Data;
  TEMPERATURE_Data[ HTS221_T_0 ].pExtData       = 0;

  *handle = (void *)&TEMPERATURE_SensorHandle[ HTS221_T_0 ];

  driver = (TEMPERATURE_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init(void **handle)
{
  TEMPERATURE_Drv_t *driver = NULL;

  if (TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].who_am_i      = LPS22HB_WHO_AM_I_VAL;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].address       = LPS22HB_ADDRESS_HIGH;         /* rhf LPS22HB_ADDRESS_LOW; */
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].instance      = LPS22HB_T_0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isEnabled     = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isCombo       = 1;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pData         = (void *)&TEMPERATURE_Data[ LPS22HB_T_0 ];
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pVTable       = (void *)&LPS22HB_T_Drv;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pExtVTable    = (void *)&LPS22HB_T_ExtDrv;

  LPS22HB_T_0_Data.comboData = &LPS22HB_Combo_Data[0];
  TEMPERATURE_Data[ LPS22HB_T_0 ].pComponentData = (void *)&LPS22HB_T_0_Data;
  TEMPERATURE_Data[ LPS22HB_T_0 ].pExtData       = 0;

  *handle = (void *)&TEMPERATURE_SensorHandle[ LPS22HB_T_0 ];

  driver = (TEMPERATURE_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines for LPS22HB */
  //LPS22HB_Sensor_IO_ITConfig();

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
 * @brief Initialize LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LPS22HB_PRESSURE_Init(void **handle)
{
  PRESSURE_Drv_t *driver = NULL;

  if (PRESSURE_SensorHandle[ LPS22HB_P_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].who_am_i      = LPS22HB_WHO_AM_I_VAL;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].address       = LPS22HB_ADDRESS_HIGH;    /* rhf LPS22HB_ADDRESS_LOW; */
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].instance      = LPS22HB_P_0;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].isInitialized = 0;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].isEnabled     = 0;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].isCombo       = 1;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].pData         = (void *)&PRESSURE_Data[ LPS22HB_P_0 ];
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].pVTable       = (void *)&LPS22HB_P_Drv;
  PRESSURE_SensorHandle[ LPS22HB_P_0 ].pExtVTable    = (void *)&LPS22HB_P_ExtDrv;

  LPS22HB_P_0_Data.comboData = &LPS22HB_Combo_Data[0];
  PRESSURE_Data[ LPS22HB_P_0 ].pComponentData = (void *)&LPS22HB_P_0_Data;
  PRESSURE_Data[ LPS22HB_P_0 ].pExtData       = 0;

  *handle = (void *)&PRESSURE_SensorHandle[ LPS22HB_P_0 ];

  driver = (PRESSURE_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines for LPS22HB */
  //LPS22HB_Sensor_IO_ITConfig();

  return COMPONENT_OK;
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

	status = BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &hPRESSURE);
	if (status != COMPONENT_OK) {
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

	return status;
}


DrvStatusTypeDef CollectSensorData() {

	float HUMIDITY_Value = 0;
	float TEMPERATURE_Value = 0;
	float PRESSURE_Value = 0;
	
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

	humidity    = (uint16_t)(HUMIDITY_Value * 2);            /* in %*2     */
	temperature = (int16_t)(TEMPERATURE_Value * 10);         /* in �C * 10 */
	pressure    = (uint16_t)(PRESSURE_Value * 100 / 10);      /* in hPa / 10 */

	return status;
}

