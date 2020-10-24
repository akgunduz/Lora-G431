/*
 * sensor_board.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_BOARD_H_
#define INC_SENSOR_BOARD_H_

#include "component.h"

#define I2C_TIMEOUT_MAX    0x1000

DrvStatusTypeDef Sensor_IO_Init(void);
uint8_t Sensor_IO_Write(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToWrite);
uint8_t Sensor_IO_Read(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToRead);

/**
 * @brief  Sensor output data rate enumerator definition
 */
typedef enum
{
  ODR_LOW,
  ODR_MID_LOW,
  ODR_MID,
  ODR_MID_HIGH,
  ODR_HIGH
} SensorOdr_t;

/**
 * @brief  Sensor full scale enumerator definition
 */
typedef enum
{
  FS_LOW,
  FS_MID_LOW,
  FS_MID,
  FS_MID_HIGH,
  FS_HIGH
} SensorFs_t;

/**
 * @brief  Sensor interrupt pin enumerator definition
 */
typedef enum
{
  INT1_PIN,
  INT2_PIN
} SensorIntPin_t;

/**
 * @brief  HUMIDITY driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Hum         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} HUMIDITY_Drv_t;

/**
 * @brief  HUMIDITY data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} HUMIDITY_Data_t;

/**
 * @brief  TEMPERATURE driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Temp        ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} TEMPERATURE_Drv_t;

/**
 * @brief  TEMPERATURE data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} TEMPERATURE_Data_t;

/**
* @brief  Error code type.
*/
typedef enum {HTS221_OK = (uint8_t)0, HTS221_ERROR = !HTS221_OK} HTS221_Error_et;

/**
* @brief  State type.
*/
typedef enum {HTS221_DISABLE = (uint8_t)0, HTS221_ENABLE = !HTS221_DISABLE} HTS221_State_et;
#define IS_HTS221_State(MODE) ((MODE == HTS221_ENABLE) || (MODE == HTS221_DISABLE))

/**
* @brief  Bit status type.
*/
typedef enum {HTS221_RESET = (uint8_t)0, HTS221_SET = !HTS221_RESET} HTS221_BitStatus_et;
#define IS_HTS221_BitStatus(MODE) ((MODE == HTS221_RESET) || (MODE == HTS221_SET))

/**
* @brief  Humidity average.
*/
typedef enum
{
  HTS221_AVGH_4         = (uint8_t)0x00,         /*!< Internal average on 4 samples */
  HTS221_AVGH_8         = (uint8_t)0x01,         /*!< Internal average on 8 samples */
  HTS221_AVGH_16        = (uint8_t)0x02,         /*!< Internal average on 16 samples */
  HTS221_AVGH_32        = (uint8_t)0x03,         /*!< Internal average on 32 samples */
  HTS221_AVGH_64        = (uint8_t)0x04,         /*!< Internal average on 64 samples */
  HTS221_AVGH_128       = (uint8_t)0x05,         /*!< Internal average on 128 samples */
  HTS221_AVGH_256       = (uint8_t)0x06,         /*!< Internal average on 256 samples */
  HTS221_AVGH_512       = (uint8_t)0x07          /*!< Internal average on 512 samples */
} HTS221_Avgh_et;
#define IS_HTS221_AVGH(AVGH) ((AVGH == HTS221_AVGH_4) || (AVGH == HTS221_AVGH_8) || \
                              (AVGH == HTS221_AVGH_16) || (AVGH == HTS221_AVGH_32) || \
                              (AVGH == HTS221_AVGH_64) || (AVGH == HTS221_AVGH_128) || \
                              (AVGH == HTS221_AVGH_256) || (AVGH == HTS221_AVGH_512))

/**
* @brief  Temperature average.
*/
typedef enum
{
  HTS221_AVGT_2         = (uint8_t)0x00,        /*!< Internal average on 2 samples */
  HTS221_AVGT_4         = (uint8_t)0x08,        /*!< Internal average on 4 samples */
  HTS221_AVGT_8         = (uint8_t)0x10,        /*!< Internal average on 8 samples */
  HTS221_AVGT_16        = (uint8_t)0x18,        /*!< Internal average on 16 samples */
  HTS221_AVGT_32        = (uint8_t)0x20,        /*!< Internal average on 32 samples */
  HTS221_AVGT_64        = (uint8_t)0x28,        /*!< Internal average on 64 samples */
  HTS221_AVGT_128       = (uint8_t)0x30,        /*!< Internal average on 128 samples */
  HTS221_AVGT_256       = (uint8_t)0x38         /*!< Internal average on 256 samples */
} HTS221_Avgt_et;
#define IS_HTS221_AVGT(AVGT) ((AVGT == HTS221_AVGT_2) || (AVGT == HTS221_AVGT_4) || \
                              (AVGT == HTS221_AVGT_8) || (AVGT == HTS221_AVGT_16) || \
                              (AVGT == HTS221_AVGT_32) || (AVGT == HTS221_AVGT_64) || \
                              (AVGT == HTS221_AVGT_128) || (AVGT == HTS221_AVGT_256))

/**
* @brief  Output data rate configuration.
*/
typedef enum
{
  HTS221_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  HTS221_ODR_1HZ       = (uint8_t)0x01,         /*!< Output Data Rate: 1Hz */
  HTS221_ODR_7HZ       = (uint8_t)0x02,         /*!< Output Data Rate: 7Hz */
  HTS221_ODR_12_5HZ    = (uint8_t)0x03,         /*!< Output Data Rate: 12.5Hz */
} HTS221_Odr_et;
#define IS_HTS221_ODR(ODR) ((ODR == HTS221_ODR_ONE_SHOT) || (ODR == HTS221_ODR_1HZ) || \
                            (ODR == HTS221_ODR_7HZ) || (ODR == HTS221_ODR_12_5HZ))


/**
* @brief  Push-pull/Open Drain selection on DRDY pin.
*/
typedef enum
{
  HTS221_PUSHPULL   = (uint8_t)0x00,   /*!< DRDY pin in push pull */
  HTS221_OPENDRAIN  = (uint8_t)0x40    /*!< DRDY pin in open drain */
} HTS221_OutputType_et;
#define IS_HTS221_OutputType(MODE) ((MODE == HTS221_PUSHPULL) || (MODE == HTS221_OPENDRAIN))

/**
* @brief  Active level of DRDY pin.
*/
typedef enum
{
  HTS221_HIGH_LVL   = (uint8_t)0x00,   /*!< HIGH state level for DRDY pin */
  HTS221_LOW_LVL    = (uint8_t)0x80    /*!< LOW state level for DRDY pin */
} HTS221_DrdyLevel_et;
#define IS_HTS221_DrdyLevelType(MODE) ((MODE == HTS221_HIGH_LVL) || (MODE == HTS221_LOW_LVL))

/**
* @brief  Driver Version Info structure definition.
*/
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t   Point;
} HTS221_DriverVersion_st;


/**
* @brief  HTS221 Init structure definition.
*/
typedef struct
{
  HTS221_Avgh_et        avg_h;            /*!< Humidity average */
  HTS221_Avgt_et        avg_t;            /*!< Temperature average */
  HTS221_Odr_et         odr;              /*!< Output data rate */
  HTS221_State_et       bdu_status;       /*!< HTS221_ENABLE/HTS221_DISABLE the block data update */
  HTS221_State_et       heater_status;    /*!< HTS221_ENABLE/HTS221_DISABLE the internal heater */

  HTS221_DrdyLevel_et   irq_level;        /*!< HTS221_HIGH_LVL/HTS221_LOW_LVL the level for DRDY pin */
  HTS221_OutputType_et  irq_output_type;  /*!< Output configuration for DRDY pin */
  HTS221_State_et       irq_enable;       /*!< HTS221_ENABLE/HTS221_DISABLE interrupt on DRDY pin */
} HTS221_Init_st;

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup HTS221_Exported_Constants
* @{
*/

/**
* @brief  Bitfield positioning.
*/
#define HTS221_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
#define HTS221_I2C_ADDRESS  (uint8_t)0xBE

/**
* @brief  Driver version.
*/
#define HTS221_DRIVER_VERSION_MAJOR (uint8_t)1
#define HTS221_DRIVER_VERSION_MINOR (uint8_t)1
#define HTS221_DRIVER_VERSION_POINT (uint8_t)0

/**
* @addtogroup HTS221_Registers
* @{
*/


/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xBC
* 7:0 This read-only register contains the device identifier for HTS221.
* \endcode
*/
#define HTS221_WHO_AM_I_REG          (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC


/**
* @brief Humidity and temperature average mode register.
* \code
* Read/write
* Default value: 0x1B
* 7:6 Reserved.
* 5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
*
*      AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
*   ----------------------------------------------------
*       0    |   0   |   0   |    2
*       0    |   0   |   1   |    4
*       0    |   1   |   0   |    8
*       0    |   1   |   1   |    16
*       1    |   0   |   0   |    32
*       1    |   0   |   1   |    64
*       1    |   1   |   0   |    128
*       1    |   1   |   1   |    256
*
* 2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
*      AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
*   ------------------------------------------------------
*       0    |   0   |   0   |    4
*       0    |   0   |   1   |    8
*       0    |   1   |   0   |    16
*       0    |   1   |   1   |    32
*       1    |   0   |   0   |    64
*       1    |   0   |   1   |    128
*       1    |   1   |   0   |    256
*       1    |   1   |   1   |    512
*
* \endcode
*/
#define HTS221_AV_CONF_REG        (uint8_t)0x10

#define HTS221_AVGT_BIT           HTS221_BIT(3)
#define HTS221_AVGH_BIT           HTS221_BIT(0)

#define HTS221_AVGH_MASK          (uint8_t)0x07
#define HTS221_AVGT_MASK          (uint8_t)0x38

/**
* @brief Control register 1.
* \code
* Read/write
* Default value: 0x00
* 7 PD: power down control. 0 - power down mode; 1 - active mode.
* 6:3 Reserved.
* 2 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 1:0 ODR1, ODR0: output data rate selection.
*
*   ODR1  | ODR0  | Humidity output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*     0   |   0   |         one shot               |         one shot
*     0   |   1   |            1                   |            1
*     1   |   0   |            7                   |            7
*     1   |   1   |           12.5                 |           12.5
*
* \endcode
*/
#define HTS221_CTRL_REG1      (uint8_t)0x20

#define HTS221_PD_BIT          HTS221_BIT(7)
#define HTS221_BDU_BIT         HTS221_BIT(2)
#define HTS221_ODR_BIT         HTS221_BIT(0)

#define HTS221_PD_MASK        (uint8_t)0x80
#define HTS221_BDU_MASK       (uint8_t)0x04
#define HTS221_ODR_MASK       (uint8_t)0x03

/**
* @brief Control register 2.
* \code
* Read/write
* Default value: 0x00
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-cleared upon completation.
* 6:2 Reserved.
* 1 HEATHER: 0: heater enable; 1: heater disable.
* 0 ONE_SHOT: 0: waiting for start of conversion; 1: start for a new dataset. Self-cleared upon completation.
* \endcode
*/
#define HTS221_CTRL_REG2      (uint8_t)0x21

#define HTS221_BOOT_BIT        HTS221_BIT(7)
#define HTS221_HEATHER_BIT     HTS221_BIT(1)
#define HTS221_ONESHOT_BIT     HTS221_BIT(0)

#define HTS221_BOOT_MASK      (uint8_t)0x80
#define HTS221_HEATHER_MASK   (uint8_t)0x02
#define HTS221_ONE_SHOT_MASK  (uint8_t)0x01

/**
* @brief Control register 3.
* \code
* Read/write
* Default value: 0x00
* 7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull; 1: open drain.
* 5:3 Reserved.
* 2 DRDY: interrupt config. 0: disable, 1: enable.
* \endcode
*/
#define HTS221_CTRL_REG3      (uint8_t)0x22

#define HTS221_DRDY_H_L_BIT    HTS221_BIT(7)
#define HTS221_PP_OD_BIT       HTS221_BIT(6)
#define HTS221_DRDY_BIT        HTS221_BIT(2)

#define HTS221_DRDY_H_L_MASK  (uint8_t)0x80
#define HTS221_PP_OD_MASK     (uint8_t)0x40
#define HTS221_DRDY_MASK      (uint8_t)0x04

/**
* @brief  Status register.
* \code
* Read
* Default value: 0x00
* 7:2 Reserved.
* 1 H_DA: Humidity data available. 0: new data for humidity is not yet available; 1: new data for humidity is available.
* 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* \endcode
*/
#define HTS221_STATUS_REG    (uint8_t)0x27

#define HTS221_H_DA_BIT       HTS221_BIT(1)
#define HTS221_T_DA_BIT       HTS221_BIT(0)

#define HTS221_HDA_MASK      (uint8_t)0x02
#define HTS221_TDA_MASK      (uint8_t)0x01

/**
* @brief  Humidity data (LSB).
* \code
* Read
* Default value: 0x00.
* HOUT7 - HOUT0: Humidity data LSB (2's complement).
* \endcode
*/
#define HTS221_HR_OUT_L_REG        (uint8_t)0x28

/**
* @brief  Humidity data (MSB).
* \code
* Read
* Default value: 0x00.
* HOUT15 - HOUT8: Humidity data MSB (2's complement).
* \endcode
*/
#define HTS221_HR_OUT_H_REG        (uint8_t)0x29


/**
* @brief  Temperature data (LSB).
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* \endcode
*/
#define HTS221_TEMP_OUT_L_REG         (uint8_t)0x2A

/**
* @brief  Temperature data (MSB).
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* \endcode
*/
#define HTS221_TEMP_OUT_H_REG         (uint8_t)0x2B

/**
* @brief  Calibration registers.
* \code
* Read
* \endcode
*/
#define HTS221_H0_RH_X2        (uint8_t)0x30
#define HTS221_H1_RH_X2        (uint8_t)0x31
#define HTS221_T0_DEGC_X8      (uint8_t)0x32
#define HTS221_T1_DEGC_X8      (uint8_t)0x33
#define HTS221_T0_T1_DEGC_H2   (uint8_t)0x35
#define HTS221_H0_T0_OUT_L     (uint8_t)0x36
#define HTS221_H0_T0_OUT_H     (uint8_t)0x37
#define HTS221_H1_T0_OUT_L     (uint8_t)0x3A
#define HTS221_H1_T0_OUT_H     (uint8_t)0x3B
#define HTS221_T0_OUT_L        (uint8_t)0x3C
#define HTS221_T0_OUT_H        (uint8_t)0x3D
#define HTS221_T1_OUT_L        (uint8_t)0x3E
#define HTS221_T1_OUT_H        (uint8_t)0x3F

#define HTS221_assert_param(expr) ((void)0)

HTS221_Error_et HTS221_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
HTS221_Error_et HTS221_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

HTS221_Error_et HTS221_Get_DriverVersion(HTS221_DriverVersion_st* version);
HTS221_Error_et HTS221_Get_DeviceID(void *handle, uint8_t* deviceid);

HTS221_Error_et HTS221_Set_InitConfig(void *handle, HTS221_Init_st* pxInit);
HTS221_Error_et HTS221_Get_InitConfig(void *handle, HTS221_Init_st* pxInit);
HTS221_Error_et HTS221_DeInit(void *handle);
HTS221_Error_et HTS221_IsMeasurementCompleted(void *handle, HTS221_BitStatus_et* Is_Measurement_Completed);

HTS221_Error_et HTS221_Get_Measurement(void *handle, uint16_t* humidity, int16_t* temperature);
HTS221_Error_et HTS221_Get_RawMeasurement(void *handle, int16_t* humidity, int16_t* temperature);
HTS221_Error_et HTS221_Get_Humidity(void *handle, uint16_t* value);
HTS221_Error_et HTS221_Get_HumidityRaw(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_TemperatureRaw(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_Temperature(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_DataStatus(void *handle, HTS221_BitStatus_et* humidity, HTS221_BitStatus_et* temperature);
HTS221_Error_et HTS221_Activate(void *handle);
HTS221_Error_et HTS221_DeActivate(void *handle);

HTS221_Error_et HTS221_Set_AvgHT(void *handle, HTS221_Avgh_et avgh, HTS221_Avgt_et avgt);
HTS221_Error_et HTS221_Set_AvgH(void *handle, HTS221_Avgh_et avgh);
HTS221_Error_et HTS221_Set_AvgT(void *handle, HTS221_Avgt_et avgt);
HTS221_Error_et HTS221_Get_AvgHT(void *handle, HTS221_Avgh_et* avgh, HTS221_Avgt_et* avgt);
HTS221_Error_et HTS221_Set_BduMode(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_BduMode(void *handle, HTS221_State_et* status);
HTS221_Error_et HTS221_Set_PowerDownMode(void *handle, HTS221_BitStatus_et status);
HTS221_Error_et HTS221_Get_PowerDownMode(void *handle, HTS221_BitStatus_et* status);
HTS221_Error_et HTS221_Set_Odr(void *handle, HTS221_Odr_et odr);
HTS221_Error_et HTS221_Get_Odr(void *handle, HTS221_Odr_et* odr);
HTS221_Error_et HTS221_MemoryBoot(void *handle);
HTS221_Error_et HTS221_Set_HeaterState(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_HeaterState(void *handle, HTS221_State_et* status);
HTS221_Error_et HTS221_StartOneShotMeasurement(void *handle);
HTS221_Error_et HTS221_Set_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et status);
HTS221_Error_et HTS221_Get_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et* status);
HTS221_Error_et HTS221_Set_IrqOutputType(void *handle, HTS221_OutputType_et value);
HTS221_Error_et HTS221_Get_IrqOutputType(void *handle, HTS221_OutputType_et* value);
HTS221_Error_et HTS221_Set_IrqEnable(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_IrqEnable(void *handle, HTS221_State_et* status);

#define HTS221_SENSORS_MAX_NUM  1     /**< HTS221 max number of instances */
#define HTS221_ADDRESS_DEFAULT  0xBE  /**< HTS221 I2C Address */

typedef struct
{
  uint8_t isHumInitialized;
  uint8_t isTempInitialized;
  uint8_t isHumEnabled;
  uint8_t isTempEnabled;
} HTS221_Combo_Data_t;

/**
 * @brief HTS221 humidity specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_H_Data_t;


/**
 * @brief HTS221 temperature specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_T_Data_t;

extern HUMIDITY_Drv_t HTS221_H_Drv;
extern TEMPERATURE_Drv_t HTS221_T_Drv;
extern HTS221_Combo_Data_t HTS221_Combo_Data[HTS221_SENSORS_MAX_NUM];

/**
* @brief  Error type.
*/
typedef enum {LPS22HB_OK = (uint8_t)0, LPS22HB_ERROR = !LPS22HB_OK} LPS22HB_Error_et;

/**
* @brief  Enable/Disable type.
*/
typedef enum {LPS22HB_DISABLE = (uint8_t)0, LPS22HB_ENABLE = !LPS22HB_DISABLE} LPS22HB_State_et;
#define IS_LPS22HB_State(MODE) ((MODE == LPS22HB_ENABLE) || (MODE == LPS22HB_DISABLE) )

/**
* @brief  Bit status type.
*/
typedef enum {LPS22HB_RESET = (uint8_t)0, LPS22HB_SET = !LPS22HB_RESET} LPS22HB_BitStatus_et;
#define IS_LPS22HB_BitStatus(MODE) ((MODE == LPS22HB_RESET) || (MODE == LPS22HB_SET))

/*RES_CONF see LC_EN bit*/
/**
* @brief  LPS22HB Power/Noise Mode configuration.
*/
typedef enum
{
  LPS22HB_LowNoise   =  (uint8_t)0x00,       /*!< Low Noise mode */
  LPS22HB_LowPower   =  (uint8_t)0x01        /*!< Low Current mode */
} LPS22HB_PowerMode_et;

#define IS_LPS22HB_PowerMode(MODE) ((MODE == LPS22HB_LowNoise) || (MODE == LPS22HB_LowPower))

/**
* @brief  Output data rate configuration.
*/
typedef enum
{

  LPS22HB_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS22HB_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS22HB_ODR_10HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  LPS22HB_ODR_25HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  LPS22HB_ODR_50HZ      = (uint8_t)0x40,          /*!< Output Data Rate: 50Hz */
  LPS22HB_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} LPS22HB_Odr_et;

#define IS_LPS22HB_ODR(ODR) ((ODR == LPS22HB_ODR_ONE_SHOT) || (ODR == LPS22HB_ODR_1HZ) || \
                             (ODR == LPS22HB_ODR_10HZ) || (ODR == LPS22HB_ODR_25HZ)|| (ODR == LPS22HB_ODR_50HZ) || (ODR == LPS22HB_ODR_75HZ))

/**
* @brief  Low Pass Filter Cutoff Configuration.
*/
typedef enum
{

  LPS22HB_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  LPS22HB_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} LPS22HB_LPF_Cutoff_et;

#define IS_LPS22HB_LPF_Cutoff(CUTOFF) ((CUTOFF == LPS22HB_ODR_9) || (CUTOFF == LPS22HB_ODR_20) )

/**
* @brief  Block data update.
*/

typedef enum
{
  LPS22HB_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
  LPS22HB_BDU_NO_UPDATE             =  (uint8_t)0x02   /*!< Data updated after a read operation */
} LPS22HB_Bdu_et;
#define IS_LPS22HB_BDUMode(MODE) ((MODE == LPS22HB_BDU_CONTINUOUS_UPDATE) || (MODE == LPS22HB_BDU_NO_UPDATE))

/**
* @brief  LPS22HB Spi Mode configuration.
*/
typedef enum
{
  LPS22HB_SPI_4_WIRE   =  (uint8_t)0x00,
  LPS22HB_SPI_3_WIRE   =  (uint8_t)0x01
} LPS22HB_SPIMode_et;

#define IS_LPS22HB_SPIMode(MODE) ((MODE == LPS22HB_SPI_4_WIRE) || (MODE == LPS22HB_SPI_3_WIRE))


/**
* @brief  LPS22HB Interrupt Active Level Configuration (on High or Low)
*/
typedef enum
{
  LPS22HB_ActiveHigh = (uint8_t)0x00,
  LPS22HB_ActiveLow  = (uint8_t)0x80
} LPS22HB_InterruptActiveLevel_et;
#define IS_LPS22HB_InterruptActiveLevel(MODE) ((MODE == LPS22HB_ActiveHigh) || (MODE == LPS22HB_ActiveLow))

/**
* @brief  LPS22HB Push-pull/Open Drain selection on Interrupt pads.
*/
typedef enum
{
  LPS22HB_PushPull = (uint8_t)0x00,
  LPS22HB_OpenDrain  = (uint8_t)0x40
} LPS22HB_OutputType_et;
#define IS_LPS22HB_OutputType(MODE) ((MODE == LPS22HB_PushPull) || (MODE == LPS22HB_OpenDrain))


/**
* @brief  Data Signal on INT pad control bits.
*/
typedef enum
{
  LPS22HB_DATA = (uint8_t)0x00,
  LPS22HB_P_HIGH = (uint8_t)0x01,
  LPS22HB_P_LOW = (uint8_t)0x02,
  LPS22HB_P_LOW_HIGH = (uint8_t)0x03
} LPS22HB_OutputSignalConfig_et;
#define IS_LPS22HB_OutputSignal(MODE) ((MODE == LPS22HB_DATA) || (MODE == LPS22HB_P_HIGH)||\
                                       (MODE == LPS22HB_P_LOW) || (MODE == LPS22HB_P_LOW_HIGH))



/**
* @brief  LPS22HB Interrupt Differential Status.
*/

typedef struct
{
  uint8_t PH;          /*!< High Differential Pressure event occured */
  uint8_t PL;          /*!< Low Differential Pressure event occured */
  uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */
  uint8_t BOOT;        /*!< i '1' indicates that the Boot (Reboot) phase is running */
} LPS22HB_InterruptDiffStatus_st;


/**
* @brief  LPS22HB Pressure and Temperature data status.
*/
typedef struct
{
  uint8_t TempDataAvailable;           /*!< Temperature data available bit */
  uint8_t PressDataAvailable;          /*!< Pressure data available bit */
  uint8_t TempDataOverrun;             /*!< Temperature data over-run bit */
  uint8_t PressDataOverrun;            /*!< Pressure data over-run bit */
} LPS22HB_DataStatus_st;


/**
* @brief  LPS22HB Clock Tree  configuration.
*/
typedef enum
{
  LPS22HB_CTE_NotBalanced   =  (uint8_t)0x00,
  LPS22HB_CTE_Balanced   =  (uint8_t)0x20
} LPS22HB_CTE_et;

#define IS_LPS22HB_CTE(MODE) ((MODE == LPS22HB_CTE_NotBalanced) || (MODE == LPS22HB_CTE_Balanced))

/**
* @brief  LPS22HB Fifo Mode.
*/

typedef enum
{
  LPS22HB_FIFO_BYPASS_MODE                    = (uint8_t)0x00,    /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS22HB_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS22HB_FifoMode_et;

#define IS_LPS22HB_FifoMode(MODE) ((MODE == LPS22HB_FIFO_BYPASS_MODE) || (MODE ==LPS22HB_FIFO_MODE)||\
                                   (MODE == LPS22HB_FIFO_STREAM_MODE) || (MODE == LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE)||\
                                   (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) ||  (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE))


/**
* @brief  LPS22HB Fifo Satus.
*/
typedef struct
{
  uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level: 00000: FIFO empty; 10000: FIFO is FULL and ha 32 unread samples  */
  uint8_t FIFO_EMPTY;          /*!< Empty FIFO Flag .1 FIFO is empty (see FIFO_level) */
  uint8_t FIFO_FULL;          /*!< Full FIFO flag.1 FIFO is Full (see FIFO_level) */
  uint8_t FIFO_OVR;           /*!< Overrun bit status. 1 FIFO is full and at least one sample in the FIFO has been overwritten */
  uint8_t FIFO_FTH;            /*!< FIFO Threshold (Watermark) Status. 1 FIFO filling is equal or higher then FTH (wtm) level.*/
} LPS22HB_FifoStatus_st;



/**
* @brief  LPS22HB Configuration structure definition.
*/
typedef struct
{
  LPS22HB_PowerMode_et   PowerMode;                    /*!< Enable Low Current Mode (low Power) or Low Noise Mode*/
  LPS22HB_Odr_et         OutputDataRate;                /*!< Output Data Rate */
  LPS22HB_Bdu_et
  BDU;                         /*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
  LPS22HB_State_et     LowPassFilter;           /*!< Enable/ Disable Low Pass Filter */
  LPS22HB_LPF_Cutoff_et  LPF_Cutoff;                    /*!< Low Pass Filter Configuration */
  LPS22HB_SPIMode_et   Sim;               /*!< SPI Serial Interface Mode selection */
  LPS22HB_State_et
  IfAddInc;                       /*!< Enable/Disable Register address automatically inceremented during a multiple byte access */
} LPS22HB_ConfigTypeDef_st;


/**
* @brief  LPS22HB Interrupt structure definition .
*/
typedef struct
{
  LPS22HB_InterruptActiveLevel_et     INT_H_L;                /*!< Interrupt active high, low. Default value: 0 */
  LPS22HB_OutputType_et       PP_OD;            /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
  LPS22HB_OutputSignalConfig_et
  OutputSignal_INT; /*!< Data signal on INT Pad: Data,Pressure High, Preessure Low,P High or Low*/
  LPS22HB_State_et
  DRDY;                   /*!< Enable/Disable Data Ready Interrupt on INT_DRDY Pin*/
  LPS22HB_State_et
  FIFO_OVR;                /*!< Enable/Disable FIFO Overrun Interrupt on INT_DRDY Pin*/
  LPS22HB_State_et
  FIFO_FTH;                /*!< Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.*/
  LPS22HB_State_et
  FIFO_FULL;               /*!< Enable/Disable FIFO FULL interrupt on INT_DRDY pin.*/
  LPS22HB_State_et        LatchIRQ;   /*!< Latch Interrupt request in to INT_SOURCE reg*/
  int16_t           THS_threshold;    /*!< Threshold value for pressure interrupt generation*/
  LPS22HB_State_et       AutoRifP;                                      /*!< Enable/Disable  AutoRifP function */
  LPS22HB_State_et       AutoZero;                                      /*!< Enable/Disable  AutoZero function */
} LPS22HB_InterruptTypeDef_st;

/**
* @brief  LPS22HB FIFO structure definition.
*/
typedef struct
{
  LPS22HB_FifoMode_et       FIFO_MODE;               /*!< Fifo Mode Selection */
  LPS22HB_State_et      WTM_INT;    /*!< Enable/Disable the watermark interrupt*/
  uint8_t         WTM_LEVEL;    /*!< FIFO threshold/Watermark level selection*/
} LPS22HB_FIFOTypeDef_st;

#define IS_LPS22HB_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))
/**
* @brief  LPS22HB Measure Type definition.
*/
typedef struct
{
  int16_t Tout;
  int32_t Pout;
} LPS22HB_MeasureTypeDef_st;


/**
* @brief  LPS22HB Driver Version Info structure definition.
*/
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t Point;
} LPS22HB_DriverVersion_st;


/**
* @brief  Bitfield positioning.
*/
#define LPS22HB_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
/* SD0/SA0(pin 5) is connected to the voltage supply*/
//#define LPS22HB_ADDRESS  (uint8_t)0xBA
/*SDO/SA0 (pin5) is connected to the GND*/
#define LPS22HB_ADDRESS  (uint8_t)0xB8

/**
* @brief  Set the LPS22HB driver version.
*/

#define LPS22HB_DriverVersion_Major (uint8_t)1
#define LPS22HB_DriverVersion_Minor (uint8_t)0
#define LPS22HB_DriverVersion_Point (uint8_t)0

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup LPS22HB_Exported_Constants
* @{
*/


/**
* @addtogroup LPS22HB_Register
* @{
*/



/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xB1
* 7:0 This read-only register contains the device identifier that, for LPS22HB, is set to B1h.
* \endcode
*/

#define LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1


/**
* @brief Reference Pressure  Register(LSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL7-0: Lower part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_XL_REG         (uint8_t)0x15


/**
* @brief Reference Pressure Register (Middle data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL15-8: Middle part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_L_REG          (uint8_t)0x16

/**
* @brief Reference Pressure Register (MSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL23-16 Higest part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_H_REG          (uint8_t)0x17


/**
* @brief Pressure and temperature resolution mode Register
* \code
* Read/write
* Default value: 0x05
* 7:2 These bits must be set to 0 for proper operation of the device
* 1: Reserved
* 0 LC_EN: Low Current Mode Enable. Default 0
* \endcode
*/
#define LPS22HB_RES_CONF_REG     (uint8_t)0x1A

#define LPS22HB_LCEN_MASK        (uint8_t)0x01

/**
* @brief Control Register 1
* \code
* Read/write
* Default value: 0x00
* 7: This bit must be set to 0 for proper operation of the device
* 6:4 ODR2, ODR1, ODR0: output data rate selection.Default 000
*     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*      0    |  0    |  0    |         one shot               |         one shot
*      0    |  0    |  1    |            1                   |            1
*      0    |  1    |  0    |            10                  |           10
*      0    |  1    |  1    |            25                  |           25
*      1    |  0    |  0    |            50                  |           50
*      1    |  0    |  1    |            75                  |         75
*      1    |  1    |  0    |         Reserved               |         Reserved
*      1    |  1    |  1    |         Reserved               |         Reserved
*
* 3 EN_LPFP: Enable Low Pass filter on Pressure data. Default value:0
* 2:LPF_CFG Low-pass configuration register. (0: Filter cutoff is ODR/9; 1: filter cutoff is ODR/20)
* 1 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire
* \endcode
*/
#define LPS22HB_CTRL_REG1      (uint8_t)0x10

#define LPS22HB_ODR_MASK                (uint8_t)0x70
#define LPS22HB_LPFP_MASK               (uint8_t)0x08
#define LPS22HB_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define LPS22HB_BDU_MASK                (uint8_t)0x02
#define LPS22HB_SIM_MASK                (uint8_t)0x01

#define LPS22HB_LPFP_BIT    LPS22HB_BIT(3)


/**
* @brief Control  Register 2
* \code
* Read/write
* Default value: 0x10
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completation
* 6 FIFO_EN: FIFO Enable. 0: disable; 1:  enable
* 5 STOP_ON_FTH: Stop on FIFO Threshold  FIFO Watermark level use. 0: disable; 1: enable
* 4 IF_ADD_INC: Register address automatically incrementeed during a multiple byte access with a serial interface (I2C or SPI). Default value 1.( 0: disable; 1: enable)
* 3 I2C DIS:  Disable I2C interface 0: I2C Enabled; 1: I2C disabled
* 2 SWRESET: Software reset. 0: normal mode; 1: SW reset. Self-clearing upon completation
* 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
* 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
* \endcode
*/
#define LPS22HB_CTRL_REG2      (uint8_t)0x11

#define LPS22HB_BOOT_BIT       LPS22HB_BIT(7)
#define LPS22HB_FIFO_EN_BIT    LPS22HB_BIT(6)
#define LPS22HB_WTM_EN_BIT     LPS22HB_BIT(5)
#define LPS22HB_ADD_INC_BIT    LPS22HB_BIT(4)
#define LPS22HB_I2C_BIT        LPS22HB_BIT(3)
#define LPS22HB_SW_RESET_BIT   LPS22HB_BIT(2)

#define LPS22HB_FIFO_EN_MASK   (uint8_t)0x40
#define LPS22HB_WTM_EN_MASK    (uint8_t)0x20
#define LPS22HB_ADD_INC_MASK   (uint8_t)0x10
#define LPS22HB_I2C_MASK       (uint8_t)0x08
#define LPS22HB_ONE_SHOT_MASK  (uint8_t)0x01


/**
* @brief CTRL Reg3 Interrupt Control Register
* \code
* Read/write
* Default value: 0x00
* 7 INT_H_L: Interrupt active high, low. 0:active high; 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
* 5 F_FSS5: FIFO full flag on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 4 F_FTH: FIFO threshold (watermark) status on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 3 F_OVR: FIFO overrun interrupt on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 2 DRDY: Data-ready signal on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 1:0 INT_S2, INT_S1: data signal on INT pad control bits.
*    INT_S2  | INT_S1  | INT pin
*   ------------------------------------------------------
*        0       |      0      |     Data signal( in order of priority:PTH_DRDY or F_FTH or F_OVR_or F_FSS5
*        0       |      1      |     Pressure high (P_high)
*        1       |      0      |     Pressure low (P_low)
*        1       |      1      |     P_low OR P_high
* \endcode
*/
#define LPS22HB_CTRL_REG3      (uint8_t)0x12

#define LPS22HB_PP_OD_BIT       LPS22HB_BIT(6)
#define LPS22HB_FIFO_FULL_BIT   LPS22HB_BIT(5)
#define LPS22HB_FIFO_FTH_BIT    LPS22HB_BIT(4)
#define LPS22HB_FIFO_OVR_BIT    LPS22HB_BIT(3)
#define LPS22HB_DRDY_BIT        LPS22HB_BIT(2)


#define LPS22HB_INT_H_L_MASK            (uint8_t)0x80
#define LPS22HB_PP_OD_MASK              (uint8_t)0x40
#define LPS22HB_FIFO_FULL_MASK          (uint8_t)0x20
#define LPS22HB_FIFO_FTH_MASK           (uint8_t)0x10
#define LPS22HB_FIFO_OVR_MASK           (uint8_t)0x08
#define LPS22HB_DRDY_MASK               (uint8_t)0x04
#define LPS22HB_INT_S12_MASK            (uint8_t)0x03


/**
* @brief Interrupt Differential configuration Register
* \code
* Read/write
* Default value: 0x00.
* 7 AUTORIFP: AutoRifP Enable
* 6 RESET_ARP: Reset AutoRifP function
* 4 AUTOZERO: Autozero enabled
* 5 RESET_AZ: Reset Autozero Function
* 3 DIFF_EN: Interrupt generation enable
* 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
* 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
* 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
* \endcode
*/
#define LPS22HB_INTERRUPT_CFG_REG  (uint8_t)0x0B

#define LPS22HB_DIFF_EN_BIT       LPS22HB_BIT(3)
#define LPS22HB_LIR_BIT           LPS22HB_BIT(2)
#define LPS22HB_PLE_BIT           LPS22HB_BIT(1)
#define LPS22HB_PHE_BIT           LPS22HB_BIT(0)

#define LPS22HB_AUTORIFP_MASK     (uint8_t)0x80
#define LPS22HB_RESET_ARP_MASK    (uint8_t)0x40
#define LPS22HB_AUTOZERO_MASK     (uint8_t)0x20
#define LPS22HB_RESET_AZ_MASK     (uint8_t)0x10
#define LPS22HB_DIFF_EN_MASK      (uint8_t)0x08
#define LPS22HB_LIR_MASK          (uint8_t)0x04
#define LPS22HB_PLE_MASK          (uint8_t)0x02
#define LPS22HB_PHE_MASK          (uint8_t)0x01



/**
* @brief Interrupt source Register (It is cleared by reading it)
* \code
* Read
* Default value: ----.
* 7 BOOT_STATUS:  If 1 indicates that the Boot (Reboot) phase is running.
* 6:3 Reserved: Keep these bits at 0
* 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
* 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
* 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
* \endcode
*/
#define LPS22HB_INTERRUPT_SOURCE_REG   (uint8_t)0x25

#define LPS22HB_BOOT_STATUS_BIT        LPS22HB_BIT(7)
#define LPS22HB_IA_BIT                 LPS22HB_BIT(2)
#define LPS22HB_PL_BIT                 LPS22HB_BIT(1)
#define LPS22HB_PH_BIT                 LPS22HB_BIT(0)

#define LPS22HB_BOOT_STATUS_MASK      (uint8_t)0x80
#define LPS22HB_IA_MASK               (uint8_t)0x04
#define LPS22HB_PL_MASK               (uint8_t)0x02
#define LPS22HB_PH_MASK               (uint8_t)0x01


/**
* @brief  Status Register
* \code
* Read
* Default value: ---
* 7:6 Reserved: 0
* 5 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
* 4 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
* 3:2 Reserved: 0
* 1 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* 0 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
* \endcode
*/
#define LPS22HB_STATUS_REG         (uint8_t)0x27

#define LPS22HB_TOR_BIT            LPS22HB_BIT(5)
#define LPS22HB_POR_BIT            LPS22HB_BIT(4)
#define LPS22HB_TDA_BIT            LPS22HB_BIT(1)
#define LPS22HB_PDA_BIT            LPS22HB_BIT(0)

#define LPS22HB_TOR_MASK           (uint8_t)0x20
#define LPS22HB_POR_MASK           (uint8_t)0x10
#define LPS22HB_TDA_MASK           (uint8_t)0x02
#define LPS22HB_PDA_MASK           (uint8_t)0x01



/**
* @brief  Pressure data (LSB) register.
* \code
* Read
* Default value: 0x00.(To be verified)
* POUT7 - POUT0: Pressure data LSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/

#define LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
/**
* @brief  Pressure data (Middle part) register.
* \code
* Read
* Default value: 0x80.
* POUT15 - POUT8: Pressure data middle part (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS22HB_PRESS_OUT_L_REG        (uint8_t)0x29

/**
* @brief  Pressure data (MSB) register.
* \code
* Read
* Default value: 0x2F.
* POUT23 - POUT16: Pressure data MSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS22HB_PRESS_OUT_H_REG        (uint8_t)0x2A

/**
* @brief  Temperature data (LSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS22HB_TEMP_OUT_L_REG         (uint8_t)0x2B

/**
* @brief  Temperature data (MSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS22HBH_TEMP_OUT_H_REG         (uint8_t)0x2C

/**
* @brief Threshold pressure (LSB) register.
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS22HB_THS_P_LOW_REG           (uint8_t)0x0C

/**
* @brief Threshold pressure (MSB)
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS22HB_THS_P_HIGH_REG         (uint8_t)0x0D

/**
* @brief FIFO control register
* \code
* Read/write
* Default value: 0x00
* 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
*     FM2   | FM1   | FM0   |    FIFO MODE
*   ---------------------------------------------------
*      0    |  0    |  0    | BYPASS MODE
*      0    |  0    |  1    | FIFO MODE. Stops collecting data when full
*      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO
*      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE
*      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE
*      1    |  0    |  1    | Reserved for future use
*      1    |  1    |  0    | Reserved
*      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE
*
* 4:0 WTM_POINT4-0 : FIFO Watermark level selection (0-31)
*/
#define LPS22HB_CTRL_FIFO_REG          (uint8_t)0x14

#define LPS22HB_FIFO_MODE_MASK        (uint8_t)0xE0
#define LPS22HB_WTM_POINT_MASK        (uint8_t)0x1F


/**
* @brief FIFO Status register
* \code
* Read
* Default value: ----
* 7 FTH_FIFO: FIFO threshold status. 0:FIFO filling is lower than FTH level; 1: FIFO is equal or higher than FTH level.
* 6 OVR: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full and at least one sample in the FIFO has been overwritten.
* 5:0 FSS: FIFO Stored data level. 000000: FIFO empty, 100000: FIFO is full and has 32 unread samples.
* \endcode
*/
#define LPS22HB_STATUS_FIFO_REG        (uint8_t)0x26

#define LPS22HB_FTH_FIFO_BIT          LPS22HB_BIT(7)
#define LPS22HB_OVR_FIFO_BIT          LPS22HB_BIT(6)

#define LPS22HB_FTH_FIFO_MASK         (uint8_t)0x80
#define LPS22HB_OVR_FIFO_MASK         (uint8_t)0x40
#define LPS22HB_LEVEL_FIFO_MASK       (uint8_t)0x3F
#define LPS22HB_FIFO_EMPTY            (uint8_t)0x00
#define LPS22HB_FIFO_FULL             (uint8_t)0x20



/**
* @brief Pressure offset register  (LSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS7-0:Pressure Offset for 1 point calibration (OPC) after soldering.
* This register contains the low part of the pressure offset value after soldering,for
* differential pressure computing. The complete value is given by RPDS_L & RPDS_H
* and is expressed as signed 2 complement value.
* \endcode
*/
#define LPS22HB_RPDS_L_REG        (uint8_t)0x18

/**
* @brief Pressure offset register (MSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS15-8:Pressure Offset for 1 point calibration  (OPC) after soldering.
* This register contains the high part of the pressure offset value after soldering (see description RPDS_L)
* \endcode
*/
#define LPS22HB_RPDS_H_REG        (uint8_t)0x19


/**
* @brief Clock Tree Configuration register
* \code
* Read/write
* Default value: 0x00
* 7:6 Reserved.
* 5: CTE: Clock Tree Enhancement
* \endcode
*/

#define LPS22HB_CLOCK_TREE_CONFIGURATION        (uint8_t)0x43

#define LPS22HB_CTE_MASK           (uint8_t)0x20

#define LPS22HB_assert_param(expr) ((void)0)

LPS22HB_Error_et LPS22HB_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
LPS22HB_Error_et LPS22HB_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

/**
* @brief  Init the HAL layer.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
#define LPS22HB_HalInit  (LPS22HB_Error_et)HAL_Init_I2C

/**
* @brief  DeInit the HAL layer.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
#define LPS22HB_HalDeInit  (LPS22HB_Error_et)HAL_DeInit_I2C

LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version);
LPS22HB_Error_et LPS22HB_Init(void *handle);
LPS22HB_Error_et LPS22HB_DeInit(void *handle);
LPS22HB_Error_et LPS22HB_Get_DeviceID(void *handle, uint8_t* deviceid);
LPS22HB_Error_et LPS22HB_Set_PowerMode(void *handle, LPS22HB_PowerMode_et mode);
LPS22HB_Error_et LPS22HB_Get_PowerMode(void *handle, LPS22HB_PowerMode_et* mode);
LPS22HB_Error_et LPS22HB_Set_Odr(void *handle, LPS22HB_Odr_et odr);
LPS22HB_Error_et LPS22HB_Get_Odr(void *handle, LPS22HB_Odr_et* odr);
LPS22HB_Error_et LPS22HB_Set_LowPassFilter(void *handle, LPS22HB_State_et state);
LPS22HB_Error_et LPS22HB_Set_LowPassFilterCutoff(void *handle, LPS22HB_LPF_Cutoff_et cutoff);
LPS22HB_Error_et LPS22HB_Set_Bdu(void *handle, LPS22HB_Bdu_et bdu);
LPS22HB_Error_et LPS22HB_Get_Bdu(void *handle, LPS22HB_Bdu_et* bdu);
LPS22HB_Error_et LPS22HB_Set_SpiInterface(void *handle, LPS22HB_SPIMode_et spimode);
LPS22HB_Error_et LPS22HB_Get_SpiInterface(void *handle, LPS22HB_SPIMode_et* spimode);
LPS22HB_Error_et LPS22HB_SwReset(void *handle);
LPS22HB_Error_et LPS22HB_MemoryBoot(void *handle);
LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void *handle);
LPS22HB_Error_et LPS22HB_Set_FifoModeUse(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_AutomaticIncrementRegAddress(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void *handle);
LPS22HB_Error_et LPS22HB_Set_I2C(void *handle, LPS22HB_State_et i2cstate);
LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(void *handle, LPS22HB_InterruptActiveLevel_et mode);
LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(void *handle, LPS22HB_OutputType_et output);
LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(void *handle, LPS22HB_OutputSignalConfig_et config);
LPS22HB_Error_et LPS22HB_Set_DRDYInterrupt(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_FIFO_OVR_Interrupt(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_FIFO_FTH_Interrupt(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_FIFO_FULL_Interrupt(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_AutoRifP(void *handle);
LPS22HB_Error_et LPS22HB_ResetAutoRifP(void *handle);
LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void *handle);
LPS22HB_Error_et LPS22HB_ResetAutoZeroFunction(void *handle);
LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et diff_en) ;
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et* diff_en);
LPS22HB_Error_et LPS22HB_LatchInterruptRequest(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_PLE(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Set_PHE(void *handle, LPS22HB_State_et status);
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(void *handle,
    LPS22HB_InterruptDiffStatus_st* interruptsource);
LPS22HB_Error_et LPS22HB_Get_DataStatus(void *handle, LPS22HB_DataStatus_st* datastatus);
LPS22HB_Error_et LPS22HB_Get_RawPressure(void *handle, int32_t *raw_press);
LPS22HB_Error_et LPS22HB_Get_Pressure(void *handle, int32_t* Pout);
LPS22HB_Error_et LPS22HB_Get_RawTemperature(void *handle, int16_t *raw_data);
LPS22HB_Error_et LPS22HB_Get_Temperature(void *handle, int16_t* Tout);
LPS22HB_Error_et LPS22HB_Get_PressureThreshold(void *handle, int16_t *P_ths);
LPS22HB_Error_et LPS22HB_Set_PressureThreshold(void *handle, int16_t P_ths);
LPS22HB_Error_et LPS22HB_Set_FifoMode(void *handle, LPS22HB_FifoMode_et fifomode);
LPS22HB_Error_et LPS22HB_Get_FifoMode(void *handle, LPS22HB_FifoMode_et* fifomode);
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel);
LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel);
LPS22HB_Error_et LPS22HB_Get_FifoStatus(void *handle, LPS22HB_FifoStatus_st* status);
LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(void *handle, int16_t *pressoffset);
LPS22HB_Error_et LPS22HB_Get_ReferencePressure(void *handle, int32_t* RefP);
LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(void *handle, uint8_t* Is_Measurement_Completed);
LPS22HB_Error_et LPS22HB_Get_Measurement(void *handle, LPS22HB_MeasureTypeDef_st *Measurement_Value);
LPS22HB_Error_et LPS22HB_Set_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);
LPS22HB_Error_et LPS22HB_Get_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);
LPS22HB_Error_et LPS22HB_Set_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt);
LPS22HB_Error_et LPS22HB_Get_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt);
LPS22HB_Error_et LPS22HB_Set_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);
LPS22HB_Error_et LPS22HB_Get_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);
LPS22HB_Error_et LPS22HB_Set_ClockTreeConfifuration(void *handle, LPS22HB_CTE_et mode);

#define LPS22HB_SENSORS_MAX_NUM  1     /**< LPS22HB max number of instances */

#define LPS22HB_ADDRESS_LOW      0xB8  /**< LPS22HB I2C Address Low */
#define LPS22HB_ADDRESS_HIGH     0xBA  /**< LPS22HB I2C Address High */

/**
 * @brief LPS22HB pressure extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Ovr_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Fth_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Stop_On_Fth         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Usage               ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data            ) ( DrvContextTypeDef*, float*, float* );
  DrvStatusTypeDef ( *FIFO_Get_Mode            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Watermark_Level ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Watermark_Usage     ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Interrupt       ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Reset_Interrupt     ) ( DrvContextTypeDef*, uint8_t );
} LPS22HB_P_ExtDrv_t;

/**
 * @brief LPS22HB temperature extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Ovr_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Fth_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Stop_On_Fth         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Usage               ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data            ) ( DrvContextTypeDef*, float*, float* );
  DrvStatusTypeDef ( *FIFO_Get_Mode            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Watermark_Level ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Watermark_Usage     ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Interrupt       ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Reset_Interrupt     ) ( DrvContextTypeDef*, uint8_t );
} LPS22HB_T_ExtDrv_t;

/**
 * @brief LPS22HB combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isPressInitialized;
  uint8_t isTempInitialized;
  uint8_t isPressEnabled;
  uint8_t isTempEnabled;
  float Last_ODR;
} LPS22HB_Combo_Data_t;

/**
 * @brief LPS22HB pressure specific data internal structure definition
 */
typedef struct
{
  LPS22HB_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HB_P_Data_t;

/**
 * @brief LPS22HB temperature specific data internal structure definition
 */
typedef struct
{
  LPS22HB_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HB_T_Data_t;

/**
 * @brief  PRESSURE driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Press       ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} PRESSURE_Drv_t;

/**
 * @brief  PRESSURE data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} PRESSURE_Data_t;

extern PRESSURE_Drv_t LPS22HB_P_Drv;
extern TEMPERATURE_Drv_t LPS22HB_T_Drv;
extern LPS22HB_Combo_Data_t LPS22HB_Combo_Data[LPS22HB_SENSORS_MAX_NUM];
extern LPS22HB_P_ExtDrv_t LPS22HB_P_ExtDrv;
extern LPS22HB_T_ExtDrv_t LPS22HB_T_ExtDrv;

#endif /* INC_SENSOR_BOARD_H_ */
