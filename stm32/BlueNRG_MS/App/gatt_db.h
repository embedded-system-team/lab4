/**
  ******************************************************************************
  * @file    App/gatt_db.h
  * @brief   Custom Accelerometer GATT Service declarations
  ******************************************************************************
  */

#ifndef GATT_DB_H
#define GATT_DB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bluenrg_def.h"
#include "bluenrg_gatt_aci.h"

/* Exported variables --------------------------------------------------------*/
extern uint16_t AccServiceHandle;
extern uint16_t AccDataCharHandle;   /* Char_A: Accel Data */
extern uint16_t AccFreqCharHandle;   /* Char_B: Sampling Frequency */

/**
 * @brief ODR enum index definition
 *
 * RPi writes one of these values to Char_B.
 * STM32 maps the index to the corresponding LSM6DSL ODR setting
 * and RTOS task delay period.
 *
 * Index | ODR      | Period
 * ------|----------|---------
 *   0   | 12.5 Hz  |  80 ms
 *   1   |  26 Hz   |  38 ms
 *   2   |  52 Hz   |  19 ms
 *   3   | 104 Hz   |   9 ms
 */
typedef enum {
  ACC_ODR_12HZ5 = 0,
  ACC_ODR_26HZ  = 1,
  ACC_ODR_52HZ  = 2,
  ACC_ODR_104HZ = 3,
  ACC_ODR_MAX   = 3
} AccODR_Index_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Add the custom Accelerometer GATT service (Char_A + Char_B).
 * @retval BLE_STATUS_SUCCESS or BLE_STATUS_ERROR
 */
tBleStatus Add_AccService(void);

/**
 * @brief  Update Char_A value and send BLE notification to subscribed client.
 * @param  x  X-axis in mg
 * @param  y  Y-axis in mg
 * @param  z  Z-axis in mg
 * @retval BLE_STATUS_SUCCESS or BLE_STATUS_ERROR
 */
tBleStatus AccData_Update(int16_t x, int16_t y, int16_t z);

/**
 * @brief  Handle READ request on Char_A.
 * @param  conn_handle  Current BLE connection handle
 * @param  x, y, z      Latest acceleration values
 */
void AccData_ReadRequestCB(uint16_t conn_handle, int16_t x, int16_t y, int16_t z);

#ifdef __cplusplus
}
#endif

#endif /* GATT_DB_H */