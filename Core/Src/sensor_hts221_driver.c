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
#include "sensor_hts221_board.h"
#include "sensor_driver.h"
#include "i2c.h"

static HTS221_H_Data_t HTS221_H_0_Data; /* Humidity - sensor 0. */
static HTS221_T_Data_t HTS221_T_0_Data; /* Temperature - sensor 0 HTS221 on board. */

DrvStatusTypeDef BSP_HTS221_HUMIDITY_Init(void **handle)
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

/**
 * @brief Initialize HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_HTS221_TEMPERATURE_Init(void **handle)
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

