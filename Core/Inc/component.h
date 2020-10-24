/*
 * component.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_COMPONENT_H_
#define INC_COMPONENT_H_

#include <stdint.h>
#include <string.h>
/**
 * @brief  NULL pointer definition
 */
#ifndef NULL
#define NULL ( void * )0
#endif

/**
 * @brief  Component's Context structure definition.
 */
typedef struct
{

  /* Identity */
  uint8_t who_am_i;

  /* Configuration */
  uint8_t address;       /* Sensor I2C address (NOTE: Not a unique sensor ID). */
  uint8_t instance;      /* Sensor instance (NOTE: Sensor ID unique only within its class). */
  uint8_t isInitialized; /* Sensor setup done. */
  uint8_t isEnabled;     /* Sensor ON. */
  uint8_t isCombo;       /* Combo sensor (component consists of more sensors). */

  /* Pointer to the Data */
  void *pData;

  /* Pointer to the Virtual Table */
  void *pVTable;
  /* Pointer to the Extended Virtual Table */
  void *pExtVTable;
} DrvContextTypeDef;

/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED
} DrvStatusTypeDef;

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* INC_COMPONENT_H_ */
