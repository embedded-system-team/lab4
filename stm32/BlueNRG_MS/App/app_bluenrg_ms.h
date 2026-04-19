/**
  ******************************************************************************
  * @file    App/app_bluenrg_ms.h
  * @brief   BlueNRG-MS application declarations
  ******************************************************************************
  */

#ifndef APP_BLUENRG_MS_H
#define APP_BLUENRG_MS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hci.h"
#include "gatt_db.h"

/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t  ble_connected;
extern volatile uint16_t connection_handle;
extern volatile uint8_t  set_connectable;

/* Exported functions --------------------------------------------------------*/
void MX_BlueNRG_MS_Init(void);
void MX_BlueNRG_MS_Process(void);
void user_notify(void *pData);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLUENRG_MS_H */