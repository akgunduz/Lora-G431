/**
 ******************************************************************************
 * @file    accelero.c
 * @author  MEMS Application Team
 * @version V4.1.0
 * @date    01-November-2017
 * @brief   This file provides a set of functions needed to manage the accelerometer sensor
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "LSM303AGR_ACC.h"
#include "sensor_board.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2 X_NUCLEO_IKS01A2
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO Accelerometer
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
static ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.
static LSM303AGR_X_Data_t LSM303AGR_X_0_Data;                     // Accelerometer - sensor 2.

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_LSM303AGR_ACCELERO_Init( void **handle );

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Public_Functions Public functions
 * @{
 */

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
    default:
    case ACCELERO_SENSORS_AUTO:
      /* Try to init accelerometers with following priority order: */
      if ( BSP_LSM303AGR_ACCELERO_Init(handle) == COMPONENT_OK )
      {
        return COMPONENT_OK;
      }
      break;

    case LSM303AGR_X_0:
      return BSP_LSM303AGR_ACCELERO_Init(handle);
  }

  return COMPONENT_ERROR;
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
 * @brief Check if LSM303AGR sensor is available
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LSM303AGR_AVAIL_Ext( void **handle )
{
  ACCELERO_Drv_t *driver = NULL;
  
  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ TEST_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ TEST_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
  ACCELERO_SensorHandle[ TEST_X_0 ].instance      = LSM303AGR_X_0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ TEST_X_0 ].pData         = ( void * )&ACCELERO_Data[ LSM303AGR_X_0 ];
  ACCELERO_SensorHandle[ TEST_X_0 ].pVTable       = ( void * )&LSM303AGR_X_Drv;
  ACCELERO_SensorHandle[ TEST_X_0 ].pExtVTable    = ( void * )&LSM303AGR_X_ExtDrv;
  ACCELERO_Data[ TEST_X_0 ].pComponentData        = ( void * )&LSM303AGR_X_0_Data;
  ACCELERO_Data[ TEST_X_0 ].pExtData              = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ TEST_X_0 ];
  driver = ( ACCELERO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Check_WhoAmI( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  memset((*handle), 0, sizeof(DrvContextTypeDef));
  *handle = NULL;
  
  return COMPONENT_OK;
}

/**
 * @brief Initialize an LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LSM303AGR_ACCELERO_Init( void **handle )
{
  ACCELERO_Drv_t *driver = NULL;

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].instance      = LSM303AGR_X_0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pData         = ( void * )&ACCELERO_Data[ LSM303AGR_X_0 ];
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pVTable       = ( void * )&LSM303AGR_X_Drv;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pExtVTable    = ( void * )&LSM303AGR_X_ExtDrv;

  LSM303AGR_X_0_Data.comboData = &LSM303AGR_Combo_Data[0];
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
  
  /* Configure interrupt lines for LSM303AGR */
//  LSM303AGR_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

