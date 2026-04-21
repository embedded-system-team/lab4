/**
  ******************************************************************************
  * @file    App/gatt_db.h
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
extern uint16_t AccDataCharHandle;    /* Char_A: Accel Data   READ | NOTIFY */
extern uint16_t AccFreqCharHandle;    /* Char_B: Sampling Freq READ | WRITE */
extern uint16_t MotionCharHandle;     /* Char_C: Motion Event  NOTIFY only  */

/* ODR index definition */
typedef enum {
  ACC_ODR_12HZ5 = 0,
  ACC_ODR_26HZ  = 1,
  ACC_ODR_52HZ  = 2,
  ACC_ODR_104HZ = 3,
  ACC_ODR_MAX   = 3
} AccODR_Index_t;

/* Exported functions --------------------------------------------------------*/
tBleStatus Add_AccService(void);
tBleStatus AccData_Update(int16_t x, int16_t y, int16_t z);
void       AccData_ReadRequestCB(uint16_t conn_handle,
                                 int16_t x, int16_t y, int16_t z);
tBleStatus Motion_Notify(void);

#ifdef __cplusplus
}
#endif

#endif /* GATT_DB_H */