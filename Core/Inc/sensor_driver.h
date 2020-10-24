/*
 * sensor_driver.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_DRIVER_H_
#define INC_SENSOR_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include "sensor_board.h"

typedef enum
{
  HUMIDITY_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  HTS221_H_0                     /* Default on board. */
} HUMIDITY_ID_t;

#define HUMIDITY_SENSORS_MAX_NUM 1

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_HUMIDITY_Init(HUMIDITY_ID_t id, void **handle);
DrvStatusTypeDef BSP_HUMIDITY_DeInit(void **handle);
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Enable(void *handle);
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Disable(void *handle);
DrvStatusTypeDef BSP_HUMIDITY_IsInitialized(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_HUMIDITY_IsEnabled(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_HUMIDITY_IsCombo(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_HUMIDITY_Get_Instance(void *handle, uint8_t *instance);
DrvStatusTypeDef BSP_HUMIDITY_Get_WhoAmI(void *handle, uint8_t *who_am_i);
DrvStatusTypeDef BSP_HUMIDITY_Check_WhoAmI(void *handle);
DrvStatusTypeDef BSP_HUMIDITY_Get_Hum(void *handle, float *humidity);
DrvStatusTypeDef BSP_HUMIDITY_Get_ODR(void *handle, float *odr);
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR(void *handle, SensorOdr_t odr);
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR_Value(void *handle, float odr);
DrvStatusTypeDef BSP_HUMIDITY_Read_Reg(void *handle, uint8_t reg, uint8_t *data);
DrvStatusTypeDef BSP_HUMIDITY_Write_Reg(void *handle, uint8_t reg, uint8_t data);
DrvStatusTypeDef BSP_HUMIDITY_Get_DRDY_Status(void *handle, uint8_t *status);

typedef enum
{
  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  HTS221_T_0,                    /* HTS221 Default on board. */
  LPS22HB_T_0                    /* LPS22HB DIL24 adapter. */
} TEMPERATURE_ID_t;

#define TEMPERATURE_SENSORS_MAX_NUM 2

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_TEMPERATURE_Init(TEMPERATURE_ID_t id, void **handle);
DrvStatusTypeDef BSP_TEMPERATURE_DeInit(void **handle);
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Enable(void *handle);
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Disable(void *handle);
DrvStatusTypeDef BSP_TEMPERATURE_IsInitialized(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_IsEnabled(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_IsCombo(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_Get_Instance(void *handle, uint8_t *instance);
DrvStatusTypeDef BSP_TEMPERATURE_Get_WhoAmI(void *handle, uint8_t *who_am_i);
DrvStatusTypeDef BSP_TEMPERATURE_Check_WhoAmI(void *handle);
DrvStatusTypeDef BSP_TEMPERATURE_Get_Temp(void *handle, float *temperature);
DrvStatusTypeDef BSP_TEMPERATURE_Get_ODR(void *handle, float *odr);
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR(void *handle, SensorOdr_t odr);
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR_Value(void *handle, float odr);
DrvStatusTypeDef BSP_TEMPERATURE_Read_Reg(void *handle, uint8_t reg, uint8_t *data);
DrvStatusTypeDef BSP_TEMPERATURE_Write_Reg(void *handle, uint8_t reg, uint8_t data);
DrvStatusTypeDef BSP_TEMPERATURE_Get_DRDY_Status(void *handle, uint8_t *status);

DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Data_Ext(void *handle, float *pressure, float *temperature);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint8_t *nSamples);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Mode_Ext(void *handle, uint8_t mode);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext(void *handle, uint8_t interrupt);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext(void *handle, uint8_t interrupt);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext(void *handle, uint8_t watermark);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext(void *handle, uint8_t status);
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Usage_Ext(void *handle, uint8_t status);

typedef enum
{
  PRESSURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LPS22HB_P_0                 /* DIL24 adapter. */
} PRESSURE_ID_t;

#define PRESSURE_SENSORS_MAX_NUM 1

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_PRESSURE_Init(PRESSURE_ID_t id, void **handle);
DrvStatusTypeDef BSP_PRESSURE_DeInit(void **handle);
DrvStatusTypeDef BSP_PRESSURE_Sensor_Enable(void *handle);
DrvStatusTypeDef BSP_PRESSURE_Sensor_Disable(void *handle);
DrvStatusTypeDef BSP_PRESSURE_IsInitialized(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_IsEnabled(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_IsCombo(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_Get_Instance(void *handle, uint8_t *instance);
DrvStatusTypeDef BSP_PRESSURE_Get_WhoAmI(void *handle, uint8_t *who_am_i);
DrvStatusTypeDef BSP_PRESSURE_Check_WhoAmI(void *handle);
DrvStatusTypeDef BSP_PRESSURE_Get_Press(void *handle, float *pressure);
DrvStatusTypeDef BSP_PRESSURE_Get_ODR(void *handle, float *odr);
DrvStatusTypeDef BSP_PRESSURE_Set_ODR(void *handle, SensorOdr_t odr);
DrvStatusTypeDef BSP_PRESSURE_Set_ODR_Value(void *handle, float odr);
DrvStatusTypeDef BSP_PRESSURE_Read_Reg(void *handle, uint8_t reg, uint8_t *data);
DrvStatusTypeDef BSP_PRESSURE_Write_Reg(void *handle, uint8_t reg, uint8_t data);
DrvStatusTypeDef BSP_PRESSURE_Get_DRDY_Status(void *handle, uint8_t *status);

DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Full_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Fth_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Ovr_Status_Ext(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Data_Ext(void *handle, float *pressure, float *temperature);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint8_t *nSamples);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Mode_Ext(void *handle, uint8_t mode);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Interrupt_Ext(void *handle, uint8_t interrupt);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Reset_Interrupt_Ext(void *handle, uint8_t interrupt);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Set_Watermark_Level_Ext(void *handle, uint8_t watermark);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Stop_On_Fth_Ext(void *handle, uint8_t status);
DrvStatusTypeDef BSP_PRESSURE_FIFO_Usage_Ext(void *handle, uint8_t status);

DrvStatusTypeDef SensorDevicesInit();
DrvStatusTypeDef CollectSensorData();

extern uint16_t pressure;
extern int16_t temperature;
extern uint16_t humidity;


#endif /* INC_SENSOR_DRIVER_H_ */
