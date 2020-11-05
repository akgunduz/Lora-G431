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
 * @brief  Sensor axes raw data structure definition
 */
typedef struct
{
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} SensorAxesRaw_t;

/**
 * @brief  Sensor axes data structure definition
 */
typedef struct
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;

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

/**
 * @brief  MAGNETOMETER driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Axes        ) ( DrvContextTypeDef*, SensorAxes_t* );
  DrvStatusTypeDef ( *Get_AxesRaw     ) ( DrvContextTypeDef*, SensorAxesRaw_t* );
  DrvStatusTypeDef ( *Get_Sensitivity ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_FS          ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_FS          ) ( DrvContextTypeDef*, SensorFs_t );
  DrvStatusTypeDef ( *Set_FS_Value    ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} MAGNETO_Drv_t;

/**
 * @brief  MAGNETOMETER data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} MAGNETO_Data_t;


/**
 * @brief  ACCELEROMETER driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Axes        ) ( DrvContextTypeDef*, SensorAxes_t* );
  DrvStatusTypeDef ( *Get_AxesRaw     ) ( DrvContextTypeDef*, SensorAxesRaw_t* );
  DrvStatusTypeDef ( *Get_Sensitivity ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_FS          ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_FS          ) ( DrvContextTypeDef*, SensorFs_t );
  DrvStatusTypeDef ( *Set_FS_Value    ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_Axes_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Axes_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} ACCELERO_Drv_t;

/**
 * @brief  ACCELEROMETER data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} ACCELERO_Data_t;

typedef enum
{
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS,
  ALL_ACTIVE
} ACTIVE_AXIS_t;

typedef enum
{
  NORMAL_MODE,
  HIGH_RES_MODE,
  LOW_PWR_MODE
} OP_MODE_t;

typedef enum
{
  INT1_DRDY_DISABLED,
  INT1_DRDY_ENABLED
} INT1_DRDY_CONFIG_t;

typedef enum
{
  DRDY_PULSED,
  DRDY_LATCHED
} DRDY_MODE_t;

/**
 * @brief  ACCELEROMETER hardware features status data structure definition
 */
typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} ACCELERO_Event_Status_t;



#endif /* INC_SENSOR_BOARD_H_ */
