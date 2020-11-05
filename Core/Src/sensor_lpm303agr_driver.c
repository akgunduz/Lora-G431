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
#include "sensor_lpm303agr_board.h"
#include "sensor_driver.h"
#include "i2c.h"

static LSM303AGR_M_Data_t LSM303AGR_M_0_Data; // Magnetometer - sensor 0.
static LSM303AGR_X_Data_t LSM303AGR_X_0_Data; // Accelerometer - sensor 0.

DrvStatusTypeDef BSP_LSM303AGR_MAGNETO_Init( void **handle )
{
  MAGNETO_Drv_t *driver = NULL;

  if(MAGNETO_SensorHandle[ LSM303AGR_M_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].who_am_i      = LSM303AGR_MAG_WHO_AM_I;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].address       = LSM303AGR_MAG_I2C_ADDRESS;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].instance      = LSM303AGR_M_0;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].isInitialized = 0;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].isEnabled     = 0;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].isCombo       = 1;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].pData         = ( void * )&MAGNETO_Data[ LSM303AGR_M_0 ];
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].pVTable       = ( void * )&LSM303AGR_M_Drv;
  MAGNETO_SensorHandle[ LSM303AGR_M_0 ].pExtVTable    = 0;

  MAGNETO_Data[ LSM303AGR_M_0 ].pComponentData = ( void * )&LSM303AGR_M_0_Data;
  MAGNETO_Data[ LSM303AGR_M_0 ].pExtData       = 0;

  *handle = (void *)&MAGNETO_SensorHandle[ LSM303AGR_M_0 ];

  driver = ( MAGNETO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

DrvStatusTypeDef BSP_LSM303AGR_ACCELERO_Init( void **handle )
{
  ACCELERO_Drv_t *driver = NULL;

  if(ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].instance      = LSM303AGR_X_0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pData         = ( void * )&ACCELERO_Data[ LSM303AGR_X_0 ];
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pVTable       = ( void * )&LSM303AGR_X_Drv;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pExtVTable    = 0;

  ACCELERO_Data[ LSM303AGR_X_0 ].pComponentData = ( void * )&LSM303AGR_X_0_Data;
  ACCELERO_Data[ LSM303AGR_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM303AGR_X_0 ];

  driver = ( ACCELERO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

