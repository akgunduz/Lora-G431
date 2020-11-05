/*
 * sensor_board.c
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#include "sensor_board.h"
#include "sensor_hts221_board.h"
#include "i2c.h"

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

