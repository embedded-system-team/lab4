/**
  ******************************************************************************
  * @file    App/app_bluenrg_ms.c
  * @brief   BlueNRG-MS initialization and BLE event handling
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_ms.h"
#include "gatt_db.h"

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "hci_const.h"
#include "link_layer.h"
#include "bluenrg_utils.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_def.h"
#include "bluenrg_aci_const.h"
#include "sm.h"
#include "sensor.h"           /* IDB04A1, IDB05A1, BDADDR_SIZE */

#include "b_l475e_iot01a1.h"
#include "cmsis_os2.h"

/* Private defines -----------------------------------------------------------*/
#undef DEVICE_NAME_LEN        /* avoid redefinition warning from bluenrg_gap.h */
#define DEVICE_NAME      "AccSensor"
#define DEVICE_NAME_LEN  9U

#define ADV_INTERVAL_MIN  0x00A0
#define ADV_INTERVAL_MAX  0x00A0

/* Private variables ---------------------------------------------------------*/
volatile uint8_t  ble_connected    = 0;
volatile uint16_t connection_handle = 0;
volatile uint8_t  set_connectable  = 1;

static uint8_t bnrg_expansion_board = IDB04A1;
static uint8_t bdaddr[BDADDR_SIZE];

extern osMutexId_t      freqMutexHandle;
extern volatile uint8_t current_odr_idx;
extern volatile int16_t latest_acc_x;
extern volatile int16_t latest_acc_y;
extern volatile int16_t latest_acc_z;

/* Private function prototypes -----------------------------------------------*/
static void BLE_SetConnectable(void);

/* =========================================================================
 * Public API
 * ========================================================================= */
void MX_BlueNRG_MS_Init(void)
{
  uint8_t  bdaddr_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int      ret;

  hci_init(user_notify, NULL);

  getBlueNRGVersion(&hwVersion, &fwVersion);
  PRINTF("BlueNRG HWver=%d FWver=%d\r\n", hwVersion, fwVersion);

  if (hwVersion > 0x30)
    bnrg_expansion_board = IDB05A1;

  hci_reset();
  HAL_Delay(300);

  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS,
                                BDADDR_SIZE, &bdaddr_len_out, bdaddr);

  if (ret || (bdaddr[5] & 0xC0) != 0xC0)
  {
    PRINTF("BLE Init: no valid static random address, generating one.\r\n");

    /* Generate address from STM32 unique device ID */
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    bdaddr[0] = (uint8_t)(uid0 & 0xFF);
    bdaddr[1] = (uint8_t)(uid0 >> 8);
    bdaddr[2] = (uint8_t)(uid0 >> 16);
    bdaddr[3] = (uint8_t)(uid1 & 0xFF);
    bdaddr[4] = (uint8_t)(uid1 >> 8);
    bdaddr[5] = (uint8_t)(uid1 >> 16) | 0xC0;  /* MSB must be 11 for static random */

    ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS,
                                    BDADDR_SIZE, bdaddr);
    if (ret)
    {
      PRINTF("BLE Init: write BD address failed 0x%02X\r\n", ret);
      while (1);
    }
    PRINTF("BLE Init: BD address set to %02X:%02X:%02X:%02X:%02X:%02X\r\n",
          bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);
  }

  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: aci_gatt_init failed 0x%02X\r\n", ret);
    while (1);
  }

  if (bnrg_expansion_board == IDB05A1)
  {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0,
                               DEVICE_NAME_LEN,
                               &service_handle,
                               &dev_name_char_handle,
                               &appearance_char_handle);
  }
  else
  {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1,
                               &service_handle,
                               &dev_name_char_handle,
                               &appearance_char_handle);
  }

  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: aci_gap_init failed 0x%02X\r\n", ret);
    while (1);
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle,
                                   0, DEVICE_NAME_LEN, (uint8_t *)DEVICE_NAME);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: set device name failed 0x%02X\r\n", ret);
    while (1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_NOT_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT, NULL,
                                     7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456, NO_BONDING);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: set auth failed 0x%02X\r\n", ret);
    while (1);
  }

  aci_hal_set_tx_power_level(1, 4);

  ret = Add_AccService();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: Add_AccService failed 0x%02X\r\n", ret);
    while (1);
  }

  PRINTF("BLE Init: complete, advertising as \"%s\"\r\n", DEVICE_NAME);
  BLE_SetConnectable();
  set_connectable = 0;
}

void MX_BlueNRG_MS_Process(void)
{
  if (set_connectable)
  {
    BLE_SetConnectable();
    set_connectable = 0;
  }
  hci_user_evt_proc();
}

/* =========================================================================
 * Private functions
 * ========================================================================= */
static void BLE_SetConnectable(void)
{
  tBleStatus ret;

  const char local_name[] = {
    AD_TYPE_COMPLETE_LOCAL_NAME,
    'A','c','c','S','e','n','s','o','r'
  };

  hci_le_set_advertise_enable(0);

  ret = aci_gap_set_discoverable(ADV_IND,
                                 ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                 STATIC_RANDOM_ADDR,
                                 NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name,
                                 0, NULL, 0, 0);
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF("BLE_SetConnectable failed: 0x%02X\r\n", ret);
  else
    PRINTF("BLE: advertising started.\r\n");
}

/* =========================================================================
 * HCI event callback
 *
 * Event routing:
 *   EVT_DISCONN_COMPLETE  → standard HCI disconnect (event_pckt->evt)
 *   EVT_LE_META_EVENT     → LE sub-events (conn complete)
 *   EVT_VENDOR            → BlueNRG vendor events (GATT write/read)
 * ========================================================================= */
void user_notify(void *pData)
{
  hci_uart_pckt  *hci_pckt   = (hci_uart_pckt *)pData;
  hci_event_pckt *event_pckt;

  if (hci_pckt->type != HCI_EVENT_PKT) return;

  event_pckt = (hci_event_pckt *)hci_pckt->data;

  switch (event_pckt->evt)
  {
    /* ---- Standard HCI: disconnection ---- */
    case EVT_DISCONN_COMPLETE:
    {
      ble_connected    = 0;
      connection_handle = 0;
      set_connectable  = 1;
      PRINTF("BLE: disconnected.\r\n");
      BSP_LED_Off(LED2);
      break;
    }

    /* ---- LE Meta Event: connection complete ---- */
    case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *meta = (evt_le_meta_event *)event_pckt->data;

      if (meta->subevent == EVT_LE_CONN_COMPLETE)
      {
        evt_le_connection_complete *cc =
          (evt_le_connection_complete *)meta->data;

        if (cc->status == BLE_STATUS_SUCCESS)
        {
          ble_connected    = 1;
          connection_handle = cc->handle;
          PRINTF("BLE: connected handle=0x%04X\r\n", connection_handle);
          BSP_LED_On(LED2);
        }
      }
      break;
    }

    /* ---- Vendor events: GATT ---- */
    case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;

      switch (blue_evt->ecode)
      {
        /* Char_B written by client */
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt =
            (evt_gatt_attr_modified_IDB05A1 *)blue_evt->data;

          if (evt->attr_handle == (AccFreqCharHandle + 1))
          {
            uint8_t new_idx = evt->att_data[0];

            if (new_idx > ACC_ODR_MAX)
            {
              PRINTF("BLE: invalid ODR index %d\r\n", new_idx);
              break;
            }

            osMutexAcquire(freqMutexHandle, osWaitForever);
            current_odr_idx = new_idx;
            osMutexRelease(freqMutexHandle);

            aci_gatt_update_char_value(AccServiceHandle,
                                       AccFreqCharHandle,
                                       0, 1, &new_idx);
            PRINTF("BLE: ODR index → %d\r\n", new_idx);
          }
          break;
        }

        /* Char_A read request */
        case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *evt =
            (evt_gatt_read_permit_req *)blue_evt->data;

          if (evt->attr_handle == (AccDataCharHandle + 1))
          {
            AccData_ReadRequestCB(connection_handle,
                                  latest_acc_x,
                                  latest_acc_y,
                                  latest_acc_z);
          }
          break;
        }

        default:
          break;
      }
      break;
    }

    default:
      break;
  }
}