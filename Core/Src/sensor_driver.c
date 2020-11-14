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
#include "sensor_driver.h"

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
 // uint32_t start = HAL_GetTick();
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
  /*    if(((HAL_GetTick() - start) > 10000))
      {
        return 3;
      }
*/
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
  //uint32_t start = HAL_GetTick();

  if(reset_INT())
  {
    return 2;
  }

  do
  {
  /*
    if((HAL_GetTick() - start) > 4000)
    {
      return 3;
    }
*/
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

	status = BSP_ACCELERO_Sensor_Enable(hACCELERO);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not enabled! \r\n");
	  return status;
	}

	status = BSP_ACCELERO_Set_INT1_DRDY_Ext( hACCELERO, INT1_DRDY_ENABLED );
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

	status = BSP_ACCELERO_Set_Active_Axis_Ext(hACCELERO, ALL_ACTIVE);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not set axis to all! \r\n");
		return status;
	}

	//HAL_Delay(100);
	/* ACC_Value[0] = X axis, ACC_Value[1] = Y axis, ACC_Value[2] = Z axis */
	BSP_ACCELERO_Get_Axes(hACCELERO, &sensValue);

	status = BSP_ACCELERO_Set_Active_Axis_Ext(hACCELERO, Z_AXIS);
	if (status != COMPONENT_OK) {
		printf("Accelero Sensor can not set axis to Z! \r\n");
		return status;
	}
/*
	if(0 != prepare_fft_data())
    {
        printf("Problem while creating FFT data \r\n");
        return 1;
    }
*/
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

