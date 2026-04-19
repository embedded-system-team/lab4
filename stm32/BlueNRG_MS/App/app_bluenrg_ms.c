/**
  ******************************************************************************
  * @file    App/app_bluenrg_ms.c
  * @brief   BlueNRG-MS initialization and BLE event handling
  *
  *          This file is responsible for:
  *          - BLE stack initialization (GATT, GAP, custom service)
  *          - GAP advertisement (set_connectable)
  *          - GAP connection / disconnection callbacks
  *          - GATT write request handling (Char_B → ODR index update)
  *          - GATT read request handling (Char_A)
  *          - Exposing MX_BlueNRG_MS_Process() for TASK_BLE to call
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_ms.h"
#include "gatt_db.h"

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "link_layer.h"
#include "bluenrg_utils.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_def.h"
#include "sm.h"

#include "b_l475e_iot01a1.h"
#include "cmsis_os2.h"

/* Private defines -----------------------------------------------------------*/
#define DEVICE_NAME          "AccSensor"
#define DEVICE_NAME_LEN      9U

/*
 * Advertising interval: 100 ms
 * BlueNRG unit = 0.625 ms → 100 / 0.625 = 160 = 0x00A0
 */
#define ADV_INTERVAL_MIN     0x00A0
#define ADV_INTERVAL_MAX     0x00A0

/* Private variables ---------------------------------------------------------*/

/* BLE connection state - read by TASK_BLE and TASK_ACC */
volatile uint8_t  ble_connected    = 0;
volatile uint16_t connection_handle = 0;

/* Flag: need to start advertising (set after init or disconnection) */
volatile uint8_t  set_connectable  = 1;

/* Board type detection */
static uint8_t bnrg_expansion_board = IDB04A1;

/* BD address read from BlueNRG */
static uint8_t bdaddr[BDADDR_SIZE];

/* FreeRTOS semaphore handle - given by EXTI ISR, taken by TASK_BLE */
extern osSemaphoreId_t bleSemHandle;

/* Current ODR index - protected by freqMutex, written here, read by TASK_ACC */
extern osMutexId_t     freqMutexHandle;
extern volatile uint8_t current_odr_idx;

/* Latest accel values for READ request - written by TASK_ACC */
extern volatile int16_t latest_acc_x;
extern volatile int16_t latest_acc_y;
extern volatile int16_t latest_acc_z;

/* Private function prototypes -----------------------------------------------*/
static void BLE_SetConnectable(void);

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialize BLE stack, GATT service and start advertising.
 *         Called once from TASK_BLE before entering the event loop.
 */
void MX_BlueNRG_MS_Init(void)
{
  uint8_t  bdaddr_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int      ret;

  /* Initialize HCI transport layer */
  hci_init(user_notify, NULL);

  /* Read BlueNRG HW/FW version */
  getBlueNRGVersion(&hwVersion, &fwVersion);
  PRINTF("BlueNRG HWver=%d FWver=%d\r\n", hwVersion, fwVersion);

  if (hwVersion > 0x30)
  {
    bnrg_expansion_board = IDB05A1;
  }

  /*
   * Reset BlueNRG before writing config data.
   * aci_hal_write_config_data() must be the first command after reset.
   */
  hci_reset();
  HAL_Delay(100);

  /* Read static random address */
  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS,
                                 BDADDR_SIZE,
                                 &bdaddr_len_out,
                                 bdaddr);
  if (ret)
  {
    PRINTF("BLE Init: read BD address failed 0x%02X\r\n", ret);
  }

  if ((bdaddr[5] & 0xC0) != 0xC0)
  {
    PRINTF("BLE Init: invalid static random address\r\n");
    while (1);
  }

  /* GATT initialization */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: aci_gatt_init failed 0x%02X\r\n", ret);
    while (1);
  }

  /* GAP initialization - peripheral role */
  if (bnrg_expansion_board == IDB05A1)
  {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1,
                               0,
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

  /* Set device name */
  ret = aci_gatt_update_char_value(service_handle,
                                   dev_name_char_handle,
                                   0,
                                   DEVICE_NAME_LEN,
                                   (uint8_t *)DEVICE_NAME);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: set device name failed 0x%02X\r\n", ret);
    while (1);
  }

  /* Set authentication requirements - no MITM, no bonding for simplicity */
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_NOT_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     NO_BONDING);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: set auth requirement failed 0x%02X\r\n", ret);
    while (1);
  }

  /* Set TX power level */
  aci_hal_set_tx_power_level(1, 4);

  /* Add custom Accelerometer GATT service */
  ret = Add_AccService();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE Init: Add_AccService failed 0x%02X\r\n", ret);
    while (1);
  }

  PRINTF("BLE Init: complete. Device name: %s\r\n", DEVICE_NAME);

  /* Start advertising immediately */
  BLE_SetConnectable();
  set_connectable = 0;
}

/**
 * @brief  Process pending BLE HCI events.
 *         Called by TASK_BLE after semaphore is given by EXTI ISR.
 */
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

/**
 * @brief  Configure GAP advertisement and start undirected advertising.
 */
static void BLE_SetConnectable(void)
{
  tBleStatus ret;

  uint8_t local_name[] = {
    AD_TYPE_COMPLETE_LOCAL_NAME,
    'A','c','c','S','e','n','s','o','r'
  };

  /* Stop any ongoing advertising before reconfiguring */
  hci_le_set_advertise_enable(0);

  /* Set advertising type and interval */
  ret = aci_gap_set_discoverable(ADV_IND,
                                 ADV_INTERVAL_MIN,
                                 ADV_INTERVAL_MAX,
                                 STATIC_RANDOM_ADDR,
                                 NO_WHITE_LIST_USE,
                                 sizeof(local_name),
                                 local_name,
                                 0,
                                 NULL,
                                 0,
                                 0);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("BLE_SetConnectable failed: 0x%02X\r\n", ret);
  }
  else
  {
    PRINTF("BLE: advertising started.\r\n");
  }
}

/* =========================================================================
 * HCI / GAP / GATT Event Callbacks
 * These are called from hci_user_evt_proc() inside TASK_BLE context.
 * ========================================================================= */

/**
 * @brief  HCI event notification callback.
 *         Dispatches GAP and GATT events.
 */
void user_notify(void *pData)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pData;

  if (hci_pckt->type != HCI_EVENT_PKT)
  {
    return;
  }

  hci_event_pckt *event_pckt = (hci_event_pckt *)hci_pckt->data;

  switch (event_pckt->evt)
  {
    /* ---- Vendor-specific events (GAP + GATT) ---- */
    case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;

      switch (blue_evt->ecode)
      {
        /* GAP: connection established */
        case EVT_BLUE_GAP_CONNECTED:
        {
          evt_le_connection_complete *cc =
            (evt_le_connection_complete *)blue_evt->data;

          ble_connected    = 1;
          connection_handle = cc->handle;
          PRINTF("BLE: connected. Handle=0x%04X\r\n", connection_handle);
          BSP_LED_On(LED2);
          break;
        }

        /* GAP: disconnection */
        case EVT_BLUE_GAP_DISCONNECTED:
        {
          ble_connected    = 0;
          connection_handle = 0;
          PRINTF("BLE: disconnected.\r\n");
          BSP_LED_Off(LED2);

          /* Restart advertising */
          set_connectable = 1;
          break;
        }

        /* GATT: attribute modified (Char_B write from client) */
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt =
            (evt_gatt_attr_modified_IDB05A1 *)blue_evt->data;

          /*
           * AccFreqCharHandle + 1 is the value attribute handle.
           * BlueNRG reports the value attribute handle on write.
           */
          if (evt->attr_handle == (AccFreqCharHandle + 1))
          {
            uint8_t new_idx = evt->att_data[0];

            /* Validate range */
            if (new_idx > ACC_ODR_MAX)
            {
              PRINTF("BLE: invalid ODR index %d, ignoring.\r\n", new_idx);
              break;
            }

            /* Update shared ODR index under mutex */
            osMutexAcquire(freqMutexHandle, osWaitForever);
            current_odr_idx = new_idx;
            osMutexRelease(freqMutexHandle);

            /* Acknowledge: update Char_B readable value */
            aci_gatt_update_char_value(AccServiceHandle,
                                       AccFreqCharHandle,
                                       0, 1, &new_idx);

            PRINTF("BLE: ODR index updated to %d\r\n", new_idx);
          }
          break;
        }

        /* GATT: read request on Char_A */
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