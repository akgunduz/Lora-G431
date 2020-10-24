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

/*******************************************************************************
* Function Name   : HTS221_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data Read
* Return      : None
*******************************************************************************/
HTS221_Error_et HTS221_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{

  if ( NumByteToRead > 1 ) RegAddr |= 0x80;

  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return HTS221_ERROR;
  else
    return HTS221_OK;
}

/*******************************************************************************
* Function Name   : HTS221_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
HTS221_Error_et HTS221_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{

  if ( NumByteToWrite > 1 ) RegAddr |= 0x80;

  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return HTS221_ERROR;
  else
    return HTS221_OK;
}

/**
* @brief  Get the version of this driver.
* @param  pxVersion pointer to a HTS221_DriverVersion_st structure that contains the version information.
*         This parameter is a pointer to @ref HTS221_DriverVersion_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DriverVersion(HTS221_DriverVersion_st* version)
{
  version->Major = HTS221_DRIVER_VERSION_MAJOR;
  version->Minor = HTS221_DRIVER_VERSION_MINOR;
  version->Point = HTS221_DRIVER_VERSION_POINT;

  return HTS221_OK;
}

/**
* @brief  Get device type ID.
* @param  *handle Device handle.
* @param  deviceid pointer to the returned device type ID.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(HTS221_ReadReg(handle, HTS221_WHO_AM_I_REG, 1, deviceid))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Initializes the HTS221 with the specified parameters in HTS221_Init_st struct.
* @param  *handle Device handle.
* @param  pxInit pointer to a HTS221_Init_st structure that contains the configuration.
*         This parameter is a pointer to @ref HTS221_Init_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_InitConfig(void *handle, HTS221_Init_st* pxInit)
{
  uint8_t buffer[3];

  HTS221_assert_param(IS_HTS221_AVGH(pxInit->avg_h));
  HTS221_assert_param(IS_HTS221_AVGT(pxInit->avg_t));
  HTS221_assert_param(IS_HTS221_ODR(pxInit->odr));
  HTS221_assert_param(IS_HTS221_State(pxInit->bdu_status));
  HTS221_assert_param(IS_HTS221_State(pxInit->heater_status));

  HTS221_assert_param(IS_HTS221_DrdyLevelType(pxInit->irq_level));
  HTS221_assert_param(IS_HTS221_OutputType(pxInit->irq_output_type));
  HTS221_assert_param(IS_HTS221_State(pxInit->irq_enable));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  buffer[0] &= ~(HTS221_AVGH_MASK | HTS221_AVGT_MASK);
  buffer[0] |= (uint8_t)pxInit->avg_h;
  buffer[0] |= (uint8_t)pxInit->avg_t;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  buffer[0] &= ~(HTS221_BDU_MASK | HTS221_ODR_MASK);
  buffer[0] |= (uint8_t)pxInit->odr;
  buffer[0] |= ((uint8_t)pxInit->bdu_status) << HTS221_BDU_BIT;

  buffer[1] &= ~HTS221_HEATHER_BIT;
  buffer[1] |= ((uint8_t)pxInit->heater_status) << HTS221_HEATHER_BIT;

  buffer[2] &= ~(HTS221_DRDY_H_L_MASK | HTS221_PP_OD_MASK | HTS221_DRDY_MASK);
  buffer[2] |= ((uint8_t)pxInit->irq_level) << HTS221_DRDY_H_L_BIT;
  buffer[2] |= (uint8_t)pxInit->irq_output_type;
  buffer[2] |= ((uint8_t)pxInit->irq_enable) << HTS221_DRDY_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Returns a HTS221_Init_st struct with the actual configuration.
* @param  *handle Device handle.
* @param  pxInit pointer to a HTS221_Init_st structure.
*         This parameter is a pointer to @ref HTS221_Init_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_InitConfig(void *handle, HTS221_Init_st* pxInit)
{
  uint8_t buffer[3];

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  pxInit->avg_h = (HTS221_Avgh_et)(buffer[0] & HTS221_AVGH_MASK);
  pxInit->avg_t = (HTS221_Avgt_et)(buffer[0] & HTS221_AVGT_MASK);

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  pxInit->odr = (HTS221_Odr_et)(buffer[0] & HTS221_ODR_MASK);
  pxInit->bdu_status = (HTS221_State_et)((buffer[0] & HTS221_BDU_MASK) >> HTS221_BDU_BIT);
  pxInit->heater_status = (HTS221_State_et)((buffer[1] & HTS221_HEATHER_MASK) >> HTS221_HEATHER_BIT);

  pxInit->irq_level = (HTS221_DrdyLevel_et)(buffer[2] & HTS221_DRDY_H_L_MASK);
  pxInit->irq_output_type = (HTS221_OutputType_et)(buffer[2] & HTS221_PP_OD_MASK);
  pxInit->irq_enable = (HTS221_State_et)((buffer[2] & HTS221_DRDY_MASK) >> HTS221_DRDY_BIT);

  return HTS221_OK;
}

/**
* @brief  De initialization function for HTS221.
*         This function put the HTS221 in power down, make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_DeInit(void *handle)
{
  uint8_t buffer[4];

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 2, buffer))
    return HTS221_ERROR;

  /* HTS221 in power down */
  buffer[0] |= 0x01 << HTS221_PD_BIT;

  /* Make HTS221 boot */
  buffer[1] |= 0x01 << HTS221_BOOT_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 2, buffer))
    return HTS221_ERROR;

  /* Dump of data output */
  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 4, buffer))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 output registers, and calculate humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity value that must be divided by 10 to get the value in [%].
* @param  temperature pointer to the returned temperature value that must be divided by 10 to get the value in ['C].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Measurement(void *handle, uint16_t* humidity, int16_t* temperature)
{
  if ( HTS221_Get_Temperature( handle, temperature ) == HTS221_ERROR ) return HTS221_ERROR;
  if ( HTS221_Get_Humidity( handle, humidity ) == HTS221_ERROR ) return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 output registers. Humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity raw value.
* @param  temperature pointer to the returned temperature raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_RawMeasurement(void *handle, int16_t* humidity, int16_t* temperature)
{
  uint8_t buffer[4];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 4, buffer))
    return HTS221_ERROR;

  *humidity = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
  *temperature = (int16_t)((((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2]);

  return HTS221_OK;
}

/**
* @brief  Read HTS221 Humidity output registers, and calculate humidity.
* @param  *handle Device handle.
* @param  Pointer to the returned humidity value that must be divided by 10 to get the value in [%].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Humidity(void *handle, uint16_t* value)
{
  int16_t H0_T0_out, H1_T0_out, H_T_out;
  int16_t H0_rh, H1_rh;
  uint8_t buffer[2];
  float   tmp_f;

  if(HTS221_ReadReg(handle, HTS221_H0_RH_X2, 2, buffer))
    return HTS221_ERROR;
  H0_rh = buffer[0] >> 1;
  H1_rh = buffer[1] >> 1;

  if(HTS221_ReadReg(handle, HTS221_H0_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H0_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_ReadReg(handle, HTS221_H1_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H1_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;
  H_T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
  tmp_f *= 10.0f;

  *value = ( tmp_f > 1000.0f ) ? 1000
           : ( tmp_f <    0.0f ) ?    0
           : ( uint16_t )tmp_f;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 humidity output registers.
* @param  *handle Device handle.
* @param  Pointer to the returned humidity raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_HumidityRaw(void *handle, int16_t* value)
{
  uint8_t buffer[2];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  *value = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);

  return HTS221_OK;
}

/**
* @brief  Read HTS221 temperature output registers, and calculate temperature.
* @param  *handle Device handle.
* @param  Pointer to the returned temperature value that must be divided by 10 to get the value in ['C].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Temperature(void *handle, int16_t *value)
{
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  uint8_t buffer[4], tmp;
  float   tmp_f;

  if(HTS221_ReadReg(handle, HTS221_T0_DEGC_X8, 2, buffer))
    return HTS221_ERROR;
  if(HTS221_ReadReg(handle, HTS221_T0_T1_DEGC_H2, 1, &tmp))
    return HTS221_ERROR;

  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16 >> 3;
  T1_degC = T1_degC_x8_u16 >> 3;

  if(HTS221_ReadReg(handle, HTS221_T0_OUT_L, 4, buffer))
    return HTS221_ERROR;

  T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];

  if(HTS221_ReadReg(handle, HTS221_TEMP_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;
  tmp_f *= 10.0f;

  *value = ( int16_t )tmp_f;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 temperature output registers.
* @param  *handle Device handle.
* @param  Pointer to the returned temperature raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_TemperatureRaw(void *handle, int16_t* value)
{
  uint8_t buffer[2];

  if(HTS221_ReadReg(handle, HTS221_TEMP_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  *value = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);

  return HTS221_OK;
}

/**
* @brief  Get the availability of new data for humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity data status [HTS221_SET/HTS221_RESET].
* @param  temperature pointer to the returned temperature data status [HTS221_SET/HTS221_RESET].
*         This parameter is a pointer to @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DataStatus(void *handle, HTS221_BitStatus_et* humidity, HTS221_BitStatus_et* temperature)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_STATUS_REG, 1, &tmp))
    return HTS221_ERROR;

  *humidity = (HTS221_BitStatus_et)((tmp & HTS221_HDA_MASK) >> HTS221_H_DA_BIT);
  *temperature = (HTS221_BitStatus_et)(tmp & HTS221_TDA_MASK);

  return HTS221_OK;
}

/**
* @brief  Exit from power down mode.
* @param  *handle Device handle.
* @param  void.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Activate(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_PD_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Put the sensor in power down mode.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_DeActivate(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PD_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}



/**
* @brief  Check if the single measurement has completed.
* @param  *handle Device handle.
* @param  tmp is set to 1, when the measure is completed
* @retval Status [HTS221_ERROR, HTS221_OK]
*/
HTS221_Error_et HTS221_IsMeasurementCompleted(void *handle, HTS221_BitStatus_et* Is_Measurement_Completed)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_STATUS_REG, 1, &tmp))
    return HTS221_ERROR;

  if((tmp & (uint8_t)(HTS221_HDA_MASK | HTS221_TDA_MASK)) == (uint8_t)(HTS221_HDA_MASK | HTS221_TDA_MASK))
    *Is_Measurement_Completed = HTS221_SET;
  else
    *Is_Measurement_Completed = HTS221_RESET;

  return HTS221_OK;
}


/**
* @brief  Set_ humidity and temperature average mode.
* @param  *handle Device handle.
* @param  avgh is the average mode for humidity, this parameter is @ref HTS221_Avgh_et.
* @param  avgt is the average mode for temperature, this parameter is @ref HTS221_Avgt_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgHT(void *handle, HTS221_Avgh_et avgh, HTS221_Avgt_et avgt)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGH(avgh));
  HTS221_assert_param(IS_HTS221_AVGT(avgt));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~(HTS221_AVGH_MASK | HTS221_AVGT_MASK);
  tmp |= (uint8_t)avgh;
  tmp |= (uint8_t)avgt;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Set humidity average mode.
* @param  *handle Device handle.
* @param  avgh is the average mode for humidity, this parameter is @ref HTS221_Avgh_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgH(void *handle, HTS221_Avgh_et avgh)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGH(avgh));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_AVGH_MASK;
  tmp |= (uint8_t)avgh;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Set temperature average mode.
* @param  *handle Device handle.
* @param  avgt is the average mode for temperature, this parameter is @ref HTS221_Avgt_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgT(void *handle, HTS221_Avgt_et avgt)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGT(avgt));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_AVGT_MASK;
  tmp |= (uint8_t)avgt;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get humidity and temperature average mode.
* @param  *handle Device handle.
* @param  avgh pointer to the returned value with the humidity average mode.
* @param  avgt pointer to the returned value with the temperature average mode.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_AvgHT(void *handle, HTS221_Avgh_et* avgh, HTS221_Avgt_et* avgt)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  *avgh = (HTS221_Avgh_et)(tmp & HTS221_AVGH_MASK);
  *avgt = (HTS221_Avgt_et)(tmp & HTS221_AVGT_MASK);

  return HTS221_OK;
}

/**
* @brief  Set block data update mode.
* @param  *handle Device handle.
* @param  status can be HTS221_ENABLE: enable the block data update, output data registers are updated once both MSB and LSB are read.
* @param  status can be HTS221_DISABLE: output data registers are continuously updated.
*         This parameter is a @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_BduMode(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_BDU_MASK;
  tmp |= ((uint8_t)status) << HTS221_BDU_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get block data update mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with block data update mode status.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_BduMode(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_BDU_MASK) >> HTS221_BDU_BIT);

  return HTS221_OK;
}

/**
* @brief  Enter or exit from power down mode.
* @param  *handle Device handle.
* @param  status can be HTS221_SET: HTS221 in power down mode.
* @param  status can be HTS221_REET: HTS221 in active mode.
*         This parameter is a @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_PowerDownMode(void *handle, HTS221_BitStatus_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_BitStatus(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PD_MASK;
  tmp |= ((uint8_t)status) << HTS221_PD_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get if HTS221 is in active mode or in power down mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with HTS221 status.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_PowerDownMode(void *handle, HTS221_BitStatus_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_BitStatus_et)((tmp & HTS221_PD_MASK) >> HTS221_PD_BIT);

  return HTS221_OK;
}

/**
* @brief  Set the output data rate mode.
* @param  *handle Device handle.
* @param  odr is the output data rate mode.
*         This parameter is a @ref HTS221_Odr_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_Odr(void *handle, HTS221_Odr_et odr)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_ODR(odr));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the output data rate mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with output data rate mode.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Odr(void *handle, HTS221_Odr_et* odr)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= HTS221_ODR_MASK;
  *odr = (HTS221_Odr_et)tmp;

  return HTS221_OK;
}

/**
* @brief  Reboot Memory Content.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_MemoryBoot(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_BOOT_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Configure the internal heater.
* @param  *handle Device handle.
* @param  The status of the internal heater [HTS221_ENABLE/HTS221_DISABLE].
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR]
*/
HTS221_Error_et HTS221_Set_HeaterState(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_HEATHER_MASK;
  tmp |= ((uint8_t)status) << HTS221_HEATHER_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the internal heater.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the internal heater [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_HeaterState(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_HEATHER_MASK) >> HTS221_HEATHER_BIT);

  return HTS221_OK;
}

/**
* @brief  Set ONE_SHOT bit to start a new conversion (ODR mode has to be 00).
*         Once the measurement is done, ONE_SHOT bit is self-cleared.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_StartOneShotMeasurement(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_ONE_SHOT_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;

}

/**
* @brief  Set level configuration of the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  status can be HTS221_LOW_LVL: active level is LOW.
* @param  status can be HTS221_HIGH_LVL: active level is HIGH.
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et value)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_DrdyLevelType(value));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_DRDY_H_L_MASK;
  tmp |= (uint8_t)value;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get level configuration of the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the level configuration [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et* value)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *value = (HTS221_DrdyLevel_et)(tmp & HTS221_DRDY_H_L_MASK);

  return HTS221_OK;
}

/**
* @brief  Set Push-pull/open drain configuration for the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  value is the output type configuration.
*         This parameter is a @ref HTS221_OutputType_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqOutputType(void *handle, HTS221_OutputType_et value)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_OutputType(value));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PP_OD_MASK;
  tmp |= (uint8_t)value;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the configuration for the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  Pointer to the returned value with output type configuration.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqOutputType(void *handle, HTS221_OutputType_et* value)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *value = (HTS221_OutputType_et)(tmp & HTS221_PP_OD_MASK);

  return HTS221_OK;
}

/**
* @brief  Enable/disable the interrupt mode.
* @param  *handle Device handle.
* @param  status is the enable/disable for the interrupt mode.
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqEnable(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_DRDY_MASK;
  tmp |= ((uint8_t)status) << HTS221_DRDY_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the interrupt mode.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the interrupt mode configuration [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqEnable(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_DRDY_MASK) >> HTS221_DRDY_BIT);

  return HTS221_OK;
}

/**
 * @brief HTS221 combo data structure definition
 */
HTS221_Combo_Data_t HTS221_Combo_Data[HTS221_SENSORS_MAX_NUM];

/**
 * @brief Enable the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Power up the device */
  if ( HTS221_Activate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Power down the device */
  if ( HTS221_DeActivate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the HTS221 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( HTS221_Get_DeviceID( (void *)handle, who_am_i ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( HTS221_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the humidity value of the HTS221 sensor
 * @param handle the device handle
 * @param humidity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_Hum( DrvContextTypeDef *handle, float *humidity )
{

  uint16_t uint16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Humidity( (void *)handle, &uint16data ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *humidity = ( float )uint16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the HTS221 sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  int16_t int16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Temperature( (void *)handle, &int16data ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *temperature = ( float )int16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  HTS221_Odr_et odr_low_level;

  if ( HTS221_Get_Odr( (void *)handle, &odr_low_level ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case HTS221_ODR_ONE_SHOT:
      *odr =  0.0f;
      break;
    case HTS221_ODR_1HZ     :
      *odr =  1.0f;
      break;
    case HTS221_ODR_7HZ     :
      *odr =  7.0f;
      break;
    case HTS221_ODR_12_5HZ  :
      *odr = 12.5f;
      break;
    default                 :
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  HTS221_Odr_et new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = HTS221_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_MID:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_HIGH:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( HTS221_Set_Odr( (void *)handle, new_odr ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  HTS221_Odr_et new_odr;

  new_odr = ( odr <= 1.0f ) ? HTS221_ODR_1HZ
            : ( odr <= 7.0f ) ? HTS221_ODR_7HZ
            :                   HTS221_ODR_12_5HZ;

  if ( HTS221_Set_Odr( (void *)handle, new_odr ) == HTS221_ERROR )
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
static DrvStatusTypeDef HTS221_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_ReadReg( (void *)handle, reg, 1, data ) == HTS221_ERROR )
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
static DrvStatusTypeDef HTS221_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_WriteReg( (void *)handle, reg, 1, &data ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Init( DrvContextTypeDef *handle )
{

  if ( HTS221_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( HTS221_DeActivate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( HTS221_Set_BduMode( (void *)handle, HTS221_ENABLE ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set default ODR */
  if ( HTS221_Set_ODR( handle, ODR_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Enable the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 temperature sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(HTS221_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 temperature sensor is still enabled. */
  /* If yes, skip the disable function, if not call disable function */
  if((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(HTS221_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Initialize the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Init( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if ((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(HTS221_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if ((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(HTS221_H_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the HTS221 humidity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return HTS221_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return HTS221_Check_WhoAmI( handle );
}


/**
 * @brief Get the humidity value of the HTS221 humidity sensor
 * @param handle the device handle
 * @param humidity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_Hum( DrvContextTypeDef *handle, float *humidity )
{

  return HTS221_Get_Hum( handle, humidity );
}


/**
 * @brief Get the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return HTS221_Get_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return HTS221_Set_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return HTS221_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_H_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_H_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  HTS221_BitStatus_et hum_status_raw;
  HTS221_BitStatus_et temp_status_raw;

  if ( HTS221_Get_DataStatus( (void *)handle, &hum_status_raw, &temp_status_raw ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( hum_status_raw )
  {
    case HTS221_SET:
      *status = 1;
      break;
    case HTS221_RESET:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Enable the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 humidity sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled == 0))
  {
    if(HTS221_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 humidity sensor is still enabled. */
  /* If yes, skip the disable function, if not call disable function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled == 0))
  {
    if(HTS221_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Initialize the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Init( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 humidity sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized == 0))
  {
    if(HTS221_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 humidity sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized == 0))
  {
    if(HTS221_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the HTS221 temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return HTS221_Get_WhoAmI( handle, who_am_i );
}

/**
 * @brief Check the WHO_AM_I ID of the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return HTS221_Check_WhoAmI( handle );
}


/**
 * @brief Get the temperature value of the HTS221 temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  return HTS221_Get_Temp( handle, temperature );
}


/**
 * @brief Get the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return HTS221_Get_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return HTS221_Set_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return HTS221_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  HTS221_BitStatus_et hum_status_raw;
  HTS221_BitStatus_et temp_status_raw;

  if ( HTS221_Get_DataStatus( (void *)handle, &hum_status_raw, &temp_status_raw ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( temp_status_raw )
  {
    case HTS221_SET:
      *status = 1;
      break;
    case HTS221_RESET:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief HTS221 humidity driver structure
 */
HUMIDITY_Drv_t HTS221_H_Drv =
{
  HTS221_H_Init,
  HTS221_H_DeInit,
  HTS221_H_Sensor_Enable,
  HTS221_H_Sensor_Disable,
  HTS221_H_Get_WhoAmI,
  HTS221_H_Check_WhoAmI,
  HTS221_H_Get_Hum,
  HTS221_H_Get_ODR,
  HTS221_H_Set_ODR,
  HTS221_H_Set_ODR_Value,
  HTS221_H_Read_Reg,
  HTS221_H_Write_Reg,
  HTS221_H_Get_DRDY_Status
};

/**
 * @brief HTS221 temperature driver structure
 */
TEMPERATURE_Drv_t HTS221_T_Drv =
{
  HTS221_T_Init,
  HTS221_T_DeInit,
  HTS221_T_Sensor_Enable,
  HTS221_T_Sensor_Disable,
  HTS221_T_Get_WhoAmI,
  HTS221_T_Check_WhoAmI,
  HTS221_T_Get_Temp,
  HTS221_T_Get_ODR,
  HTS221_T_Set_ODR,
  HTS221_T_Set_ODR_Value,
  HTS221_T_Read_Reg,
  HTS221_T_Write_Reg,
  HTS221_T_Get_DRDY_Status
};

/*******************************************************************************
* Function Name   : LPS22HB_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data Read
* Return      : None
*******************************************************************************/
LPS22HB_Error_et LPS22HB_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
  int i = 0;

  for (i = 0; i < NumByteToRead; i++ )
  {
    if( Sensor_IO_Read(handle, RegAddr + i, &Data[i], 1 ))
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/*******************************************************************************
* Function Name   : LPS22HB_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
LPS22HB_Error_et LPS22HB_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{
  int i = 0;

  for (i = 0; i < NumByteToWrite; i++ )
  {
    if( Sensor_IO_Write(handle, RegAddr + i, &Data[i], 1 ))
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
* @brief  Read identification code by WHO_AM_I register
* @param  *handle Device handle.
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(LPS22HB_ReadReg(handle, LPS22HB_WHO_AM_I_REG, 1, deviceid))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get the LPS22HB driver version.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version)
{
  Version->Major = LPS22HB_DriverVersion_Major;
  Version->Minor = LPS22HB_DriverVersion_Minor;
  Version->Point = LPS22HB_DriverVersion_Point;

  return LPS22HB_OK;
}


/**
* @brief  Set LPS22HB Low Power or Low Noise Mode Configuration
* @param  *handle Device handle.
* @param  LPS22HB_LowNoise or LPS22HB_LowPower mode
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PowerMode(void *handle, LPS22HB_PowerMode_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_PowerMode(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LCEN_MASK;
  tmp |= (uint8_t)mode;

  if(LPS22HB_WriteReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get LPS22HB Power Mode
* @param  *handle Device handle.
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PowerMode(void *handle, LPS22HB_PowerMode_et* mode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *mode = (LPS22HB_PowerMode_et)(tmp & LPS22HB_LCEN_MASK);

  return LPS22HB_OK;
}


/**
* @brief  Set LPS22HB Output Data Rate
* @param  *handle Device handle.
* @param  Output Data Rate
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_Odr(void *handle, LPS22HB_Odr_et odr)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_ODR(odr));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get LPS22HB Output Data Rate
* @param  *handle Device handle.
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Odr(void *handle, LPS22HB_Odr_et* odr)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *odr = (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Enable/Disale low-pass filter on LPS22HB pressure data
* @param  *handle Device handle.
* @param  state: enable or disable
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_LowPassFilter(void *handle, LPS22HB_State_et state)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(state));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_MASK;
  tmp |= ((uint8_t)state) << LPS22HB_LPFP_BIT;


  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Set low-pass filter cutoff configuration on LPS22HB pressure data
* @param  *handle Device handle.
* @param  Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_LowPassFilterCutoff(void *handle, LPS22HB_LPF_Cutoff_et cutoff)
{

  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_LPF_Cutoff(cutoff));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_CUTOFF_MASK;
  tmp |= (uint8_t)cutoff;



  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;

}

/**
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to �1�.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param  *handle Device handle.
* @param  LPS22HB_BDU_CONTINUOUS_UPDATE, LPS22HB_BDU_NO_UPDATE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

LPS22HB_Error_et LPS22HB_Set_Bdu(void *handle, LPS22HB_Bdu_et bdu)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_BDUMode(bdu));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_BDU_MASK;
  tmp |= ((uint8_t)bdu);


  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_OK;

  return LPS22HB_OK;
}

/**
* @brief  Get Block Data Mode
* @param  *handle Device handle.
* @param Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Bdu(void *handle, LPS22HB_Bdu_et* bdu)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *bdu = (LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param  *handle Device handle.
* @param LPS22HB_SPI_3_WIRE, LPS22HB_SPI_4_WIRE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_SpiInterface(void *handle, LPS22HB_SPIMode_et spimode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_SPIMode(spimode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_SIM_MASK;
  tmp |= (uint8_t)spimode;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Clock Tree Configuration
* @param  *handle Device handle.
* @param  LPS22HB_CTE_NotBalanced, LPS22HB_CTE_ABalanced
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_ClockTreeConfifuration(void *handle, LPS22HB_CTE_et mode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_CTE(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_CTE_MASK;
  tmp |= (uint8_t)mode;


  if(LPS22HB_WriteReg(handle, LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param  *handle Device handle.
* @param Buffet to empty with spi mode read from Sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_SpiInterface(void *handle, LPS22HB_SPIMode_et* spimode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *spimode = (LPS22HB_SPIMode_et)(tmp & LPS22HB_SIM_MASK);

  return LPS22HB_OK;
}

/**
* @brief   Software Reset. Self-clearing upon completion
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwReset(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01 << LPS22HB_SW_RESET_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Reboot Memory Content
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

LPS22HB_Error_et LPS22HB_MemoryBoot(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01 << LPS22HB_BOOT_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Software Reset ann Reboot Memory Content.
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to �1�
 + and BOOT is set to �1�; Self-clearing upon completion.
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((0x01 << LPS22HB_SW_RESET_BIT) | (0x01 << LPS22HB_BOOT_BIT));

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable FIFO Mode
* @param  *handle Device handle.
* @param LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoModeUse(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_EN_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}
/**
* @brief   Enable/Disable FIFO Watermark Level Use
* @param  *handle Device handle.
* @param   LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_EN_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_WTM_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE. Default is LPS22HB_ENABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutomaticIncrementRegAddress(void *handle, LPS22HB_State_et status)
{

  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_ADD_INC_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_ADD_INC_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;

}

/**
* @brief  Enable/Disable I2C Interface
* @param  *handle Device handle.
* @param State: LPS22HB_ENABLE (reset bit)/ LPS22HB_DISABLE (set bit)
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_I2C(void *handle, LPS22HB_State_et statei2c)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(statei2c));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /*Reset Bit->I2C Enabled*/
  tmp &= ~LPS22HB_I2C_MASK;
  tmp |= ((uint8_t)~statei2c) << LPS22HB_I2C_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set the one-shot bit in order to start acquisition when the ONE SHOT mode
*          has been selected by the ODR configuration.
* @detail  Once the measurement is done, ONE_SHOT bit will self-clear.
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= LPS22HB_ONE_SHOT_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;

}

/**
* @brief  Set Interrupt Active on High or Low Level
* @param  *handle Device handle.
* @param  LPS22HB_ActiveHigh/LPS22HB_ActiveLow
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(void *handle, LPS22HB_InterruptActiveLevel_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_InterruptActiveLevel(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_INT_H_L_MASK;
  tmp |= ((uint8_t)mode);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param  *handle Device handle.
* @param   LPS22HB_PushPull/LPS22HB_OpenDrain
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(void *handle, LPS22HB_OutputType_et output)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputType(output));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PP_OD_MASK;
  tmp |= (uint8_t)output;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Data signal on INT pad control bits.
* @param  *handle Device handle.
* @param  LPS22HB_DATA,LPS22HB_P_HIGH_LPS22HB_P_LOW,LPS22HB_P_LOW_HIGH
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(void *handle, LPS22HB_OutputSignalConfig_et config)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputSignal(config));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~(LPS22HB_INT_S12_MASK);
  tmp |= (uint8_t)config;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_DRDYInterrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DRDY_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_DRDY_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_OVR_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_OVR_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_OVR_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_FTH_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FTH_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_FTH_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_FULL_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FULL_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_FULL_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



/**
* @brief   Enable AutoRifP function
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*               the AutoRifP is slf creared.
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutoRifP(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((uint8_t)LPS22HB_AUTORIFP_MASK);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Disable AutoRifP function
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_ResetAutoRifP(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  tmp |= ((uint8_t)LPS22HB_RESET_ARP_MASK);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set AutoZero Function bit
* @detail When set to �1�, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= LPS22HB_AUTOZERO_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_ResetAutoZeroFunction(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= LPS22HB_RESET_AZ_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE,LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et diff_en)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(diff_en));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en) << LPS22HB_DIFF_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get the DIFF_EN bit value
* @param  *handle Device handle.
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et* diff_en)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *diff_en = (LPS22HB_State_et)((tmp & LPS22HB_DIFF_EN_MASK) >> LPS22HB_DIFF_EN_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_LatchInterruptRequest(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LIR_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_LIR_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



/**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PLE(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PLE_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_PLE_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PHE(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PHE_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_PHE_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param  *handle Device handle.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(void *handle,
    LPS22HB_InterruptDiffStatus_st* interruptsource)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_SOURCE_REG, 1, &tmp))
    return LPS22HB_ERROR;

  interruptsource->PH = (uint8_t)(tmp & LPS22HB_PH_MASK);
  interruptsource->PL = (uint8_t)((tmp & LPS22HB_PL_MASK) >> LPS22HB_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & LPS22HB_IA_MASK) >> LPS22HB_IA_BIT);
  interruptsource->BOOT = (uint8_t)((tmp & LPS22HB_BOOT_STATUS_MASK) >> LPS22HB_BOOT_STATUS_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Get the status of Pressure and Temperature data
* @param  *handle Device handle.
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DataStatus(void *handle, LPS22HB_DataStatus_st* datastatus)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus->PressDataAvailable = (uint8_t)(tmp & LPS22HB_PDA_MASK);
  datastatus->TempDataAvailable = (uint8_t)((tmp & LPS22HB_TDA_MASK) >> LPS22HB_PDA_BIT);
  datastatus->TempDataOverrun = (uint8_t)((tmp & LPS22HB_TOR_MASK) >> LPS22HB_TOR_BIT);
  datastatus->PressDataOverrun = (uint8_t)((tmp & LPS22HB_POR_MASK) >> LPS22HB_POR_BIT);

  return LPS22HB_OK;
}



/**
* @brief  Get the LPS22HB raw presure value
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2�s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  *handle Device handle.
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_RawPressure(void *handle, int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;
  uint8_t i;

  if(LPS22HB_ReadReg(handle, LPS22HB_PRESS_OUT_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(i = 0; i < 3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8 * i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
    tmp |= 0xFF000000;

  *raw_press = ((int32_t)tmp);

  return LPS22HB_OK;
}

/**
* @brief    Get the LPS22HB Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2�s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  *handle Device handle.
* @param      The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Pressure(void *handle, int32_t* Pout)
{
  int32_t raw_press;

  if(LPS22HB_Get_RawPressure(handle, &raw_press))
    return LPS22HB_ERROR;

  *Pout = (raw_press * 100) / 4096;

  return LPS22HB_OK;
}

/**
* @brief    Get the Raw Temperature value.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2�s complement number.
*            Tout(degC)=TEMP_OUT/100
* @param  *handle Device handle.
* @param     Buffer to empty with the temperature raw tmp.
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_RawTemperature(void *handle, int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_TEMP_OUT_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

  *raw_data = ((int16_t)tmp);

  return LPS22HB_OK;
}


/**
* @brief    Get the Temperature value in �C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2�s complement number.
*           Tout(degC)=TEMP_OUT/100
* @param  *handle Device handle.
* @param Buffer to empty with the temperature value that must be divided by 10 to get the value in �C
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Temperature(void *handle, int16_t* Tout)
{
  int16_t raw_data;

  if(LPS22HB_Get_RawTemperature(handle, &raw_data))
    return LPS22HB_ERROR;

  *Tout = (raw_data * 10) / 100;

  return LPS22HB_OK;
}

/**
* @brief    Get the threshold value used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param  *handle Device handle.
* @param    Buffer to empty with the pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureThreshold(void *handle, int16_t* P_ths)
{
  uint8_t tempReg[2];

  if(LPS22HB_ReadReg(handle, LPS22HB_THS_P_LOW_REG, 2, tempReg))
    return LPS22HB_ERROR;

  *P_ths = (((((uint16_t)tempReg[1]) << 8) + tempReg[0]) / 16);

  return LPS22HB_OK;
}

/**
* @brief    Set the threshold value  used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param  *handle Device handle.
* @param    Pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PressureThreshold(void *handle, int16_t P_ths)
{
  uint8_t buffer[2];

  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths)) >> 8);

  if(LPS22HB_WriteReg(handle, LPS22HB_THS_P_LOW_REG, 2, buffer))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo Mode.
* @param  *handle Device handle.
* @param  Fifo Mode struct
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoMode(void *handle, LPS22HB_FifoMode_et fifomode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_FifoMode(fifomode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_MODE_MASK;
  tmp |= (uint8_t)fifomode;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief    Get Fifo Mode
* @param  *handle Device handle.
* @param   buffer to empty with fifo mode tmp
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoMode(void *handle, LPS22HB_FifoMode_et* fifomode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= LPS22HB_FIFO_MODE_MASK;
  *fifomode = (LPS22HB_FifoMode_et)tmp;

  return LPS22HB_OK;
}

/**
* @brief    Set Fifo Watermark Level.
* @param  *handle Device handle.
* @param    Watermark level value [0 31]
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_WtmLevel(wtmlevel));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_POINT_MASK;
  tmp |= wtmlevel;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get FIFO Watermark Level
* @param  *handle Device handle.
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel)
{
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, wtmlevel))
    return LPS22HB_ERROR;

  *wtmlevel &= LPS22HB_WTM_POINT_MASK;

  return LPS22HB_OK;
}

/**
* @brief    Get the Fifo Status
* @param  *handle Device handle.
* @param    Status Flag: FIFO_FTH,FIFO_EMPTY,FIFO_FULL,FIFO_OVR and level of the FIFO->FIFO_LEVEL
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoStatus(void *handle, LPS22HB_FifoStatus_st* status)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  status->FIFO_FTH = (uint8_t)((tmp & LPS22HB_FTH_FIFO_MASK) >> LPS22HB_FTH_FIFO_BIT);
  status->FIFO_OVR = (uint8_t)((tmp & LPS22HB_OVR_FIFO_MASK) >> LPS22HB_OVR_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & LPS22HB_LEVEL_FIFO_MASK);

  if(status->FIFO_LEVEL == LPS22HB_FIFO_EMPTY)
    status->FIFO_EMPTY = 0x01;
  else
    status->FIFO_EMPTY = 0x00;

  if (status->FIFO_LEVEL == LPS22HB_FIFO_FULL)
    status->FIFO_FULL = 0x01;
  else
    status->FIFO_FULL = 0x00;


  return LPS22HB_OK;
}

/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param  *handle Device handle.
* @param buffer to empty with the he pressure value (hPA)
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(void *handle, int16_t *pressoffset)
{
  uint8_t buffer[2];
  int16_t raw_press;

  if(LPS22HB_ReadReg(handle, LPS22HB_RPDS_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);

  *pressoffset = (raw_press * 100) / 4096;

  return LPS22HB_OK;
}


/**
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  *handle Device handle.
* @param  Buffer to empty with reference pressure value
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_ReferencePressure(void *handle, int32_t* RefP)
{
  uint8_t buffer[3];
  uint32_t tempVal = 0;
  int32_t raw_press;
  uint8_t i;

  if(LPS22HB_ReadReg(handle, LPS22HB_REF_P_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(i = 0; i < 3; i++)
    tempVal |= (((uint32_t)buffer[i]) << (8 * i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;

  raw_press = ((int32_t)tempVal);
  *RefP = (raw_press * 100) / 4096;


  return LPS22HB_OK;
}


/**
* @brief  Check if the single measurement has completed.
* @param  *handle Device handle.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(void *handle, uint8_t* Is_Measurement_Completed)
{
  uint8_t tmp;
  LPS22HB_DataStatus_st datastatus;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus.TempDataAvailable = (uint8_t)((tmp & LPS22HB_TDA_MASK) >> LPS22HB_TDA_BIT);
  datastatus.PressDataAvailable = (uint8_t)(tmp & LPS22HB_PDA_MASK);

  *Is_Measurement_Completed = (uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));

  return LPS22HB_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  *handle Device handle.
* @param  Pressure and temperature tmp
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Measurement(void *handle, LPS22HB_MeasureTypeDef_st *Measurement_Value)
{
  int16_t Tout;
  int32_t Pout;

  if(LPS22HB_Get_Temperature(handle, &Tout))
    return LPS22HB_ERROR;

  Measurement_Value->Tout = Tout;

  if(LPS22HB_Get_Pressure(handle, &Pout))
    return LPS22HB_ERROR;

  Measurement_Value->Pout = Pout;

  return LPS22HB_OK;

}

/**
* @brief  Initialization function for LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*        NO FIFO; NO Interrupt Enabled.
* @param  *handle Device handle.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Init(void *handle)
{
  LPS22HB_ConfigTypeDef_st pLPS22HBInit;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot(handle))
    return LPS22HB_ERROR;

  pLPS22HBInit.PowerMode = LPS22HB_LowPower;
  pLPS22HBInit.OutputDataRate = LPS22HB_ODR_25HZ;
  pLPS22HBInit.LowPassFilter = LPS22HB_DISABLE;
  pLPS22HBInit.LPF_Cutoff = LPS22HB_ODR_9;
  pLPS22HBInit.BDU = LPS22HB_BDU_NO_UPDATE;
  pLPS22HBInit.IfAddInc = LPS22HB_ENABLE; //default
  pLPS22HBInit.Sim = LPS22HB_SPI_4_WIRE;

  /* Set Generic Configuration*/
  if(LPS22HB_Set_GenericConfig(handle, &pLPS22HBInit))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  De initialization function for LPS22HB.
*         This function make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_DeInit(void *handle)
{
  LPS22HB_MeasureTypeDef_st Measurement_Value;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot(handle))
    return LPS22HB_ERROR;

  /* Dump of data output */
  if(LPS22HB_Get_Measurement(handle, &Measurement_Value))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set Generic Configuration
* @param  *handle Device handle.
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS22HB_Set_PowerMode(handle, pxLPS22HBInit->PowerMode))
    return LPS22HB_ERROR;

  /* Init the Output Data Rate*/
  if(LPS22HB_Set_Odr(handle, pxLPS22HBInit->OutputDataRate))
    return LPS22HB_ERROR;

  /* BDU bit is used to inhibit the output registers update between the reading of upper and
  lower register parts. In default mode (BDU = �0�), the lower and upper register parts are
  updated continuously. If it is not sure to read faster than output data rate, it is recommended
  to set BDU bit to �1�. In this way, after the reading of the lower (upper) register part, the
  content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/

  if(LPS22HB_Set_Bdu(handle, pxLPS22HBInit->BDU))
    return LPS22HB_ERROR;

  /*Enable/Disale low-pass filter on LPS22HB pressure data*/
  if(LPS22HB_Set_LowPassFilter(handle, pxLPS22HBInit->LowPassFilter))
    return LPS22HB_ERROR;

  /* Set low-pass filter cutoff configuration*/
  if(LPS22HB_Set_LowPassFilterCutoff(handle, pxLPS22HBInit->LPF_Cutoff))
    return LPS22HB_ERROR;

  /* SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/

  if(LPS22HB_Set_SpiInterface(handle, pxLPS22HBInit->Sim))
    return LPS22HB_ERROR;

  /*Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)*/
  if(LPS22HB_Set_AutomaticIncrementRegAddress(handle, pxLPS22HBInit->IfAddInc))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}

/**
* @brief  Get Generic configuration
* @param  *handle Device handle.
* @param  Struct to empty with configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

  uint8_t tmp;

  /*Read LPS22HB_RES_CONF_REG*/
  if(LPS22HB_Get_PowerMode(handle, &pxLPS22HBInit->PowerMode))
    return LPS22HB_ERROR;

  /*Read LPS22HB_CTRL_REG1*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->OutputDataRate = (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);
  pxLPS22HBInit->BDU = (LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);
  pxLPS22HBInit->Sim = (LPS22HB_SPIMode_et)(tmp & LPS22HB_SIM_MASK);
  pxLPS22HBInit->LowPassFilter = (LPS22HB_State_et)((tmp & LPS22HB_LPFP_MASK) >> LPS22HB_LPFP_BIT);
  pxLPS22HBInit->LPF_Cutoff = (LPS22HB_LPF_Cutoff_et)(tmp & LPS22HB_LPFP_CUTOFF_MASK);

  /*Read LPS22HB_CTRL_REG2*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->IfAddInc = (LPS22HB_State_et)((tmp & LPS22HB_ADD_INC_MASK) >> LPS22HB_ADD_INC_BIT);

  return LPS22HB_OK;
}


/**
* @brief  Set Interrupt configuration
* @param  *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS22HB_Set_InterruptActiveLevel(handle, pLPS22HBInt->INT_H_L))
    return LPS22HB_ERROR;

  /* Push-pull/open drain selection on interrupt pads.*/
  if(LPS22HB_Set_InterruptOutputType(handle, pLPS22HBInt->PP_OD))
    return LPS22HB_ERROR;

  /* Set Data signal on INT pad control bits.*/
  if(LPS22HB_Set_InterruptControlConfig(handle, pLPS22HBInt->OutputSignal_INT))
    return LPS22HB_ERROR;

  /* Enable/Disable Data-ready signal on INT_DRDY pin. */
  if(LPS22HB_Set_DRDYInterrupt(handle, pLPS22HBInt->DRDY))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO overrun interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_OVR_Interrupt(handle, pLPS22HBInt->FIFO_OVR))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO Treshold interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, pLPS22HBInt->FIFO_FTH))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO FULL interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_FULL_Interrupt(handle, pLPS22HBInt->FIFO_FULL))
    return LPS22HB_ERROR;

  /* Latch Interrupt request to the INT_SOURCE register. */
  if(LPS22HB_LatchInterruptRequest(handle, pLPS22HBInt->LatchIRQ))
    return LPS22HB_ERROR;

  /* Set the threshold value  used for pressure interrupt generation (hPA). */
  if(LPS22HB_Set_PressureThreshold(handle, pLPS22HBInt->THS_threshold))
    return LPS22HB_ERROR;

  /*Enable/Disable  AutoRifP function */
  if(pLPS22HBInt->AutoRifP == LPS22HB_ENABLE)
  {
    if(LPS22HB_Set_AutoRifP(handle))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_ResetAutoRifP(handle))
      return LPS22HB_ERROR;
  }

  /*Enable/Disable AutoZero function*/
  if(pLPS22HBInt->AutoZero == LPS22HB_ENABLE)
  {
    if(LPS22HB_Set_AutoZeroFunction(handle))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_ResetAutoZeroFunction(handle))
      return LPS22HB_ERROR;
  }


  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_HIGH)
  {
    /* Enable\Disable Interrupt Generation on differential pressure high event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_LOW)
  {
    /* Enable Interrupt Generation on differential pressure Loe event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_LOW_HIGH)
  {
    /* Enable Interrupt Generation on differential pressure high event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    /* Enable\Disable Interrupt Generation on differential pressure Loe event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Disable Interrupt Generation on differential pressure High event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Disable Interrupt Generation on differential pressure Low event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
* @brief  LPS22HBGet_InterruptConfig
* @param  *handle Device handle.
* @param  Struct to empty with configuration values
* @retval S Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  uint8_t tmp;

  /*Read LPS22HB_CTRL_REG3*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->INT_H_L = (LPS22HB_InterruptActiveLevel_et)(tmp & LPS22HB_INT_H_L_MASK);
  pLPS22HBInt->PP_OD = (LPS22HB_OutputType_et)(tmp & LPS22HB_PP_OD_MASK);
  pLPS22HBInt->OutputSignal_INT = (LPS22HB_OutputSignalConfig_et)(tmp & LPS22HB_INT_S12_MASK);
  pLPS22HBInt->DRDY = (LPS22HB_State_et)((tmp & LPS22HB_DRDY_MASK) >> LPS22HB_DRDY_BIT);
  pLPS22HBInt->FIFO_OVR = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_OVR_MASK) >> LPS22HB_FIFO_OVR_BIT);
  pLPS22HBInt->FIFO_FTH = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_FTH_MASK) >> LPS22HB_FIFO_FTH_BIT);
  pLPS22HBInt->FIFO_FULL = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_FULL_MASK) >> LPS22HB_FIFO_FULL_BIT);

  /*Read LPS22HB_INTERRUPT_CFG_REG*/
  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->LatchIRQ = (LPS22HB_State_et)((tmp & LPS22HB_LIR_MASK) >> LPS22HB_LIR_BIT);

  if(LPS22HB_Get_PressureThreshold(handle, &pLPS22HBInt->THS_threshold))
    return LPS22HB_ERROR;

  //AutoRifP and Autozero are self clear //
  pLPS22HBInt->AutoRifP = LPS22HB_DISABLE;
  pLPS22HBInt->AutoZero = LPS22HB_DISABLE;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo configuration
* @param  *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{

  if(pLPS22HBFIFO->FIFO_MODE == LPS22HB_FIFO_BYPASS_MODE)
  {
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Force->Disable FIFO Watermark Level Use*/
    if(LPS22HB_Set_FifoWatermarkLevelUse(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;

    /* Force->Disable FIFO Treshold interrupt on INT_DRDY pin. */
    if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
  }
  else
  {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;

    if (pLPS22HBFIFO->WTM_INT)
    {
      /* Enable FIFO Watermark Level Use*/
      if(LPS22HB_Set_FifoWatermarkLevelUse(handle, LPS22HB_ENABLE))
        return LPS22HB_ERROR;
      /*Set Fifo Watermark Level*/
      if(LPS22HB_Set_FifoWatermarkLevel(handle, pLPS22HBFIFO->WTM_LEVEL))
        return LPS22HB_ERROR;
      /* Force->Enable FIFO Treshold interrupt on INT_DRDY pin. */
      if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, LPS22HB_ENABLE))
        return LPS22HB_ERROR;
    }
  }

  if(LPS22HB_Set_FifoMode(handle, pLPS22HBFIFO->FIFO_MODE))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get Fifo configuration
* @param  *handle Device handle.
* @param  Struct to empty with the configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /*!< Fifo Mode Selection */
  pLPS22HBFIFO->FIFO_MODE = (LPS22HB_FifoMode_et)(tmp & LPS22HB_FIFO_MODE_MASK);

  /*!< FIFO threshold/Watermark level selection*/
  pLPS22HBFIFO->WTM_LEVEL = (uint8_t)(tmp & LPS22HB_WTM_POINT_MASK);

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /*!< Enable/Disable the watermark interrupt*/
  pLPS22HBFIFO->WTM_INT = (LPS22HB_State_et)((tmp & LPS22HB_WTM_EN_MASK) >> LPS22HB_WTM_EN_BIT);


  return LPS22HB_OK;
}

/**
 * @brief LPS22HB combo data structure definition
 */
LPS22HB_Combo_Data_t LPS22HB_Combo_Data[LPS22HB_SENSORS_MAX_NUM];

/**
 * @brief Get the WHO_AM_I ID of the LPS22HB sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LPS22HB_Get_DeviceID( (void *)handle, who_am_i ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LPS22HB_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize the LPS22HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Initialize( DrvContextTypeDef *handle, LPS22HB_Combo_Data_t *combo )
{

  if ( LPS22HB_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  combo->Last_ODR = 25.0f;

  /* Set Power mode */
  if ( LPS22HB_Set_PowerMode( (void *)handle, LPS22HB_LowPower) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if( LPS22HB_Set_LowPassFilter( (void *)handle, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set low-pass filter cutoff configuration*/
  if( LPS22HB_Set_LowPassFilterCutoff( (void *)handle, LPS22HB_ODR_9) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set block data update mode */
  if ( LPS22HB_Set_Bdu( (void *)handle, LPS22HB_BDU_NO_UPDATE ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable automatic increment for multi-byte read/write */
  if( LPS22HB_Set_AutomaticIncrementRegAddress( (void *)handle, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the pressure value of the LPS22HB sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  int32_t int32data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Pressure( (void *)handle, &int32data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *pressure = ( float )int32data / 100.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the LPS22HB sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  int16_t int16data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Temperature( (void *)handle, &int16data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *temperature = ( float )int16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LPS22HB sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LPS22HB_Odr_et odr_low_level;

  if ( LPS22HB_Get_Odr( (void *)handle, &odr_low_level ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LPS22HB_ODR_ONE_SHOT:
      *odr = 0.0f;
      break;
    case LPS22HB_ODR_1HZ:
      *odr = 1.0f;
      break;
    case LPS22HB_ODR_10HZ:
      *odr = 10.0f;
      break;
    case LPS22HB_ODR_25HZ:
      *odr = 25.0f;
      break;
    case LPS22HB_ODR_50HZ:
      *odr = 50.0f;
      break;
    case LPS22HB_ODR_75HZ:
      *odr = 75.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo )
{

  LPS22HB_Odr_et new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LPS22HB_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = LPS22HB_ODR_10HZ;
      break;
    case ODR_MID:
      new_odr = LPS22HB_ODR_25HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = LPS22HB_ODR_50HZ;
      break;
    case ODR_HIGH:
      new_odr = LPS22HB_ODR_75HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_Odr( (void *)handle, new_odr ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_ODR( handle, &combo->Last_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo )
{

  switch( odr )
  {
    case ODR_LOW:
      combo->Last_ODR = 1.0f;
      break;
    case ODR_MID_LOW:
      combo->Last_ODR = 10.0f;
      break;
    case ODR_MID:
      combo->Last_ODR = 25.0f;
      break;
    case ODR_MID_HIGH:
      combo->Last_ODR = 50.0f;
      break;
    case ODR_HIGH:
      combo->Last_ODR = 75.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo )
{

  LPS22HB_Odr_et new_odr;

  new_odr = ( odr <=  1.0f ) ? LPS22HB_ODR_1HZ
            : ( odr <= 10.0f ) ? LPS22HB_ODR_10HZ
            : ( odr <= 25.0f ) ? LPS22HB_ODR_25HZ
            : ( odr <= 50.0f ) ? LPS22HB_ODR_50HZ
            :                    LPS22HB_ODR_75HZ;

  if ( LPS22HB_Set_Odr( (void *)handle, new_odr ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_ODR( handle, &combo->Last_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo )
{

  combo->Last_ODR = ( odr <=  1.0f ) ? 1.0f
                    : ( odr <= 10.0f ) ? 10.0f
                    : ( odr <= 25.0f ) ? 25.0f
                    : ( odr <= 50.0f ) ? 50.0f
                    :                    75.0f;

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
static DrvStatusTypeDef LPS22HB_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_ReadReg( (void *)handle, reg, 1, data ) == LPS22HB_ERROR )
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
static DrvStatusTypeDef LPS22HB_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_WriteReg( (void *)handle, reg, 1, &data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @}
 */

/** @addtogroup LPS22HB_Callable_Private_Functions_Ext Callable private functions for extended features
 * @{
 */

/**
 * @brief Get the FIFO_EMPTY status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_EMPTY
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_EMPTY;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_FULL status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_FULL
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_FULL;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_OVR status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Ovr_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_OVR;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_FTH status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_FTH
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Fth_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_FTH;

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO to stop on FTH interrupt
 * @param handle the device handle
 * @param status enable or disable stopping on FTH interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Stop_On_Fth( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )status )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoWatermarkLevelUse( handle, ( LPS22HB_State_et )status ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief FIFO usage
 * @param handle the device handle
 * @param status enable or disable FIFO
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Usage( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )status )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoModeUse( handle, ( LPS22HB_State_et )status ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the number of FIFO unread samples
 * @param handle the device handle
 * @param nSamples the pointer where the number of FIFO unread samples is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint8_t *nSamples )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *nSamples = status_raw.FIFO_LEVEL;

  return COMPONENT_OK;
}

/**
 * @brief Get the oldest pressure and temperature sample from the FIFO
 * @param handle the device handle
 * @param pressure the pointer where the pressure part of FIFO sample is stored
 * @param temperature the pointer where the temperature part of FIFO sample is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Data( DrvContextTypeDef *handle, float *pressure, float *temperature )
{

  if ( LPS22HB_Get_Press( handle, pressure ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_Temp( handle, temperature ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO mode
 * @param handle the device handle
 * @param mode the pointer where the FIFO mode is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Mode( DrvContextTypeDef *handle, uint8_t *mode )
{

  LPS22HB_FifoMode_et mode_raw;

  if ( LPS22HB_Get_FifoMode( handle, &mode_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *mode = ( uint8_t )mode_raw;

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO mode
 * @param handle the device handle
 * @param mode The FIFO mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_FifoMode_et )mode )
  {
    case LPS22HB_FIFO_BYPASS_MODE:
    case LPS22HB_FIFO_MODE:
    case LPS22HB_FIFO_STREAM_MODE:
    case LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE:
    case LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE:
    case LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoMode( handle, ( LPS22HB_FifoMode_et )mode ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO watermark level
 * @param handle the device handle
 * @param watermark the pointer where the FIFO watermark level is stored; values: from 0 to 31
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Watermark_Level( DrvContextTypeDef *handle, uint8_t *watermark )
{

  if ( LPS22HB_Get_FifoWatermarkLevel( handle, watermark ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO watermark level
 * @param handle the device handle
 * @param watermark The FIFO watermark level to be set; values: from 0 to 31
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint8_t watermark )
{

  if ( LPS22HB_Set_FifoWatermarkLevel( handle, watermark ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief The FIFO watermark enable or disable
 * @param handle the device handle
 * @param usage The FIFO watermark enable or disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Watermark_Usage( DrvContextTypeDef *handle, uint8_t usage )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )usage )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoWatermarkLevelUse( handle, ( LPS22HB_State_et )usage ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO interrupt
 * @param handle the device handle
 * @param interrupt The FIFO interrupt to be set; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt )
{

  switch( interrupt )
  {
    case 0:
      if ( LPS22HB_Set_FIFO_FTH_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 1:
      if ( LPS22HB_Set_FIFO_FULL_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 2:
      if ( LPS22HB_Set_FIFO_OVR_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Reset the FIFO interrupt
 * @param handle the device handle
 * @param interrupt The FIFO interrupt to be reset; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Reset_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt )
{

  switch( interrupt )
  {
    case 0:
      if ( LPS22HB_Set_FIFO_FTH_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 1:
      if ( LPS22HB_Set_FIFO_FULL_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 2:
      if ( LPS22HB_Set_FIFO_OVR_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Enable the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Sensor_Enable( DrvContextTypeDef *handle )
{
  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  if(LPS22HB_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isPressEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Sensor_Disable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HB temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if(comboData->isTempEnabled == 0)
  {
    /* Power down the device */
    if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS22HB pressure sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS22HB_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS22HB_Check_WhoAmI( handle );
}

/*
 * @brief Initialize the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Init( DrvContextTypeDef *handle )
{
  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if(comboData->isTempInitialized == 0)
  {
    if(LPS22HB_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_DeInit( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if(comboData->isTempInitialized == 0)
  {
    if(LPS22HB_P_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the pressure value of the LPS22HB pressure sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  return LPS22HB_Get_Press( handle, pressure );
}


/**
 * @brief Get the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS22HB_Get_ODR( handle, odr );
}


/**
 * @brief Set the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HB_P_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_P_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_P_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_DataStatus_st status_raw;

  if ( LPS22HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.PressDataAvailable;

  return COMPONENT_OK;
}

/**
 * @brief Enable the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Sensor_Enable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  if(LPS22HB_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Sensor_Disable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HB pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if(comboData->isPressEnabled == 0)
  {
    /* Power down the device */
    if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS22HB temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS22HB_Get_WhoAmI( handle, who_am_i );
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS22HB_Check_WhoAmI( handle );
}

/**
 * @brief Initialize the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Init( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB pressure sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if(comboData->isPressInitialized == 0)
  {
    if(LPS22HB_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Denitialize the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_DeInit( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB pressure sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if(comboData->isPressInitialized == 0)
  {
    if(LPS22HB_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the LPS22HB temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  return LPS22HB_Get_Temp( handle, temperature );
}

/**
 * @brief Get the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS22HB_Get_ODR( handle, odr );
}



/**
 * @brief Set the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HB_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_DataStatus_st status_raw;

  if ( LPS22HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.TempDataAvailable;

  return COMPONENT_OK;
}



/**
 * @brief LPS22HB pressure driver structure
 */
PRESSURE_Drv_t LPS22HB_P_Drv =
{
  LPS22HB_P_Init,
  LPS22HB_P_DeInit,
  LPS22HB_P_Sensor_Enable,
  LPS22HB_P_Sensor_Disable,
  LPS22HB_P_Get_WhoAmI,
  LPS22HB_P_Check_WhoAmI,
  LPS22HB_P_Get_Press,
  LPS22HB_P_Get_ODR,
  LPS22HB_P_Set_ODR,
  LPS22HB_P_Set_ODR_Value,
  LPS22HB_P_Read_Reg,
  LPS22HB_P_Write_Reg,
  LPS22HB_P_Get_DRDY_Status
};

/**
 * @brief LPS22HB temperature driver structure
 */
TEMPERATURE_Drv_t LPS22HB_T_Drv =
{
  LPS22HB_T_Init,
  LPS22HB_T_DeInit,
  LPS22HB_T_Sensor_Enable,
  LPS22HB_T_Sensor_Disable,
  LPS22HB_T_Get_WhoAmI,
  LPS22HB_T_Check_WhoAmI,
  LPS22HB_T_Get_Temp,
  LPS22HB_T_Get_ODR,
  LPS22HB_T_Set_ODR,
  LPS22HB_T_Set_ODR_Value,
  LPS22HB_T_Read_Reg,
  LPS22HB_T_Write_Reg,
  LPS22HB_T_Get_DRDY_Status
};

/**
 * @brief LPS22HB pressure extended features driver internal structure
 */
LPS22HB_P_ExtDrv_t LPS22HB_P_ExtDrv =
{
  LPS22HB_FIFO_Get_Empty_Status,
  LPS22HB_FIFO_Get_Full_Status,
  LPS22HB_FIFO_Get_Ovr_Status,
  LPS22HB_FIFO_Get_Fth_Status,
  LPS22HB_FIFO_Stop_On_Fth,
  LPS22HB_FIFO_Usage,
  LPS22HB_FIFO_Get_Num_Of_Samples,
  LPS22HB_FIFO_Get_Data,
  LPS22HB_FIFO_Get_Mode,
  LPS22HB_FIFO_Set_Mode,
  LPS22HB_FIFO_Get_Watermark_Level,
  LPS22HB_FIFO_Set_Watermark_Level,
  LPS22HB_FIFO_Watermark_Usage,
  LPS22HB_FIFO_Set_Interrupt,
  LPS22HB_FIFO_Reset_Interrupt
};

/**
 * @brief LPS22HB temperature extended features driver internal structure
 */
LPS22HB_T_ExtDrv_t LPS22HB_T_ExtDrv =
{
  LPS22HB_FIFO_Get_Empty_Status,
  LPS22HB_FIFO_Get_Full_Status,
  LPS22HB_FIFO_Get_Ovr_Status,
  LPS22HB_FIFO_Get_Fth_Status,
  LPS22HB_FIFO_Stop_On_Fth,
  LPS22HB_FIFO_Usage,
  LPS22HB_FIFO_Get_Num_Of_Samples,
  LPS22HB_FIFO_Get_Data,
  LPS22HB_FIFO_Get_Mode,
  LPS22HB_FIFO_Set_Mode,
  LPS22HB_FIFO_Get_Watermark_Level,
  LPS22HB_FIFO_Set_Watermark_Level,
  LPS22HB_FIFO_Watermark_Usage,
  LPS22HB_FIFO_Set_Interrupt,
  LPS22HB_FIFO_Reset_Interrupt
};
