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
#include "sensor_lps22hb_board.h"
#include "sensor_driver.h"
#include "i2c.h"

static LPS22HB_T_Data_t LPS22HB_T_0_Data; /* Temperature - sensor 1 LPS22H/B via DIL24. */
static LPS22HB_P_Data_t LPS22HB_P_0_Data; /* Pressure - sensor 0 LPS22H/B via DIL24. */

/**
 * @brief Initialize LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init(void **handle)
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
 * @brief Initialize LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LPS22HB_PRESSURE_Init(void **handle)
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

