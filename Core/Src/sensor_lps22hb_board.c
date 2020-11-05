/*
 * sensor_board.c
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#include "sensor_lps22hb_board.h"
#include "i2c.h"

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
