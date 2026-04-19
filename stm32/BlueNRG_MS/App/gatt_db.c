/**
  ******************************************************************************
  * @file    App/gatt_db.c
  * @brief   Custom Accelerometer GATT Service implementation
  *          - Char_A: Accel Data (READ | NOTIFY), 6 bytes, int16_t x3
  *          - Char_B: Sampling Frequency (READ | WRITE), 1 byte, enum index
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "gatt_db.h"
#include "bluenrg_def.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"

/* Private macros ------------------------------------------------------------*/

/** @brief Store uint16 into buffer, little-endian */
#define HOST_TO_LE_16(buf, val) \
  ( ((buf)[0] = (uint8_t)(val)), ((buf)[1] = (uint8_t)((val) >> 8)) )

/**
 * @brief Copy a 128-bit UUID into a uint8_t[16] array.
 *        Arguments are listed from most-significant byte (index 15)
 *        to least-significant byte (index 0), matching BlueNRG convention.
 */
#define COPY_UUID_128(uuid_struct, \
  uuid_15, uuid_14, uuid_13, uuid_12, \
  uuid_11, uuid_10, uuid_9,  uuid_8,  \
  uuid_7,  uuid_6,  uuid_5,  uuid_4,  \
  uuid_3,  uuid_2,  uuid_1,  uuid_0)  \
do { \
  (uuid_struct)[0]  = (uuid_0);  (uuid_struct)[1]  = (uuid_1);  \
  (uuid_struct)[2]  = (uuid_2);  (uuid_struct)[3]  = (uuid_3);  \
  (uuid_struct)[4]  = (uuid_4);  (uuid_struct)[5]  = (uuid_5);  \
  (uuid_struct)[6]  = (uuid_6);  (uuid_struct)[7]  = (uuid_7);  \
  (uuid_struct)[8]  = (uuid_8);  (uuid_struct)[9]  = (uuid_9);  \
  (uuid_struct)[10] = (uuid_10); (uuid_struct)[11] = (uuid_11); \
  (uuid_struct)[12] = (uuid_12); (uuid_struct)[13] = (uuid_13); \
  (uuid_struct)[14] = (uuid_14); (uuid_struct)[15] = (uuid_15); \
} while(0)

/*
 * Custom Accelerometer Service UUID:
 *   A0001000-74EE-43CE-B6A1-0002A5D5C51B
 *
 * Char_A (Accel Data) UUID:
 *   A0001001-74EE-43CE-B6A1-0002A5D5C51B
 *
 * Char_B (Sampling Frequency) UUID:
 *   A0001002-74EE-43CE-B6A1-0002A5D5C51B
 */
#define COPY_ACC_SERVICE_UUID(uuid_struct) \
  COPY_UUID_128(uuid_struct, \
    0xA0,0x00,0x10,0x00, \
    0x74,0xEE,0x43,0xCE, \
    0xB6,0xA1,0x00,0x02, \
    0xA5,0xD5,0xC5,0x1B)

#define COPY_ACC_DATA_CHAR_UUID(uuid_struct) \
  COPY_UUID_128(uuid_struct, \
    0xA0,0x00,0x10,0x01, \
    0x74,0xEE,0x43,0xCE, \
    0xB6,0xA1,0x00,0x02, \
    0xA5,0xD5,0xC5,0x1B)

#define COPY_ACC_FREQ_CHAR_UUID(uuid_struct) \
  COPY_UUID_128(uuid_struct, \
    0xA0,0x00,0x10,0x02, \
    0x74,0xEE,0x43,0xCE, \
    0xB6,0xA1,0x00,0x02, \
    0xA5,0xD5,0xC5,0x1B)

/*
 * Service attribute count:
 *   1 (service declaration)
 *   + 3 (Char_A declaration + value + CCCD for NOTIFY)
 *   + 2 (Char_B declaration + value)
 *   = 6
 */
#define ACC_SERVICE_MAX_ATTR_RECORDS  6U

/* Char_A payload size: int16_t x 3 axes = 6 bytes */
#define ACC_DATA_CHAR_SIZE   6U

/* Char_B payload size: uint8_t ODR enum index = 1 byte */
#define ACC_FREQ_CHAR_SIZE   1U

/* Default ODR index on startup: 1 → 26 Hz */
#define ACC_FREQ_DEFAULT_IDX 1U

/* Private variables ---------------------------------------------------------*/
uint16_t AccServiceHandle;
uint16_t AccDataCharHandle;   /* Char_A */
uint16_t AccFreqCharHandle;   /* Char_B */

/* Reuse BlueNRG UUID wrapper types */
static Service_UUID_t service_uuid;
static Char_UUID_t    char_uuid;

/* -------------------------------------------------------------------------*/

/**
 * @brief  Add the custom Accelerometer GATT service with Char_A and Char_B.
 * @retval BLE_STATUS_SUCCESS or BLE_STATUS_ERROR
 */
tBleStatus Add_AccService(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* ---- Add Service ---- */
  COPY_ACC_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);

  ret = aci_gatt_add_serv(UUID_TYPE_128,
                          service_uuid.Service_UUID_128,
                          PRIMARY_SERVICE,
                          ACC_SERVICE_MAX_ATTR_RECORDS,
                          &AccServiceHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: aci_gatt_add_serv failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Add Char_A: Accel Data (READ | NOTIFY) ---- */
  COPY_ACC_DATA_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);

  ret = aci_gatt_add_char(AccServiceHandle,
                          UUID_TYPE_128,
                          char_uuid.Char_UUID_128,
                          ACC_DATA_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16,   /* min encryption key size */
                          0,    /* fixed length */
                          &AccDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: aci_gatt_add_char (Char_A) failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Add Char_B: Sampling Frequency (READ | WRITE) ---- */
  COPY_ACC_FREQ_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);

  ret = aci_gatt_add_char(AccServiceHandle,
                          UUID_TYPE_128,
                          char_uuid.Char_UUID_128,
                          ACC_FREQ_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          16,   /* min encryption key size */
                          0,    /* fixed length */
                          &AccFreqCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: aci_gatt_add_char (Char_B) failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Set default value for Char_B ---- */
  uint8_t default_freq = ACC_FREQ_DEFAULT_IDX;
  ret = aci_gatt_update_char_value(AccServiceHandle,
                                   AccFreqCharHandle,
                                   0,
                                   ACC_FREQ_CHAR_SIZE,
                                   &default_freq);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: set default Char_B failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  PRINTF("Add_AccService: service added successfully.\r\n");
  return BLE_STATUS_SUCCESS;
}

/* -------------------------------------------------------------------------*/

/**
 * @brief  Update Char_A with new acceleration data and trigger BLE notification.
 * @param  x  X-axis acceleration in mg (int16_t)
 * @param  y  Y-axis acceleration in mg (int16_t)
 * @param  z  Z-axis acceleration in mg (int16_t)
 * @retval BLE_STATUS_SUCCESS or BLE_STATUS_ERROR
 */
tBleStatus AccData_Update(int16_t x, int16_t y, int16_t z)
{
  tBleStatus ret;
  uint8_t buff[ACC_DATA_CHAR_SIZE];

  HOST_TO_LE_16(buff + 0, x);
  HOST_TO_LE_16(buff + 2, y);
  HOST_TO_LE_16(buff + 4, z);

  ret = aci_gatt_update_char_value(AccServiceHandle,
                                   AccDataCharHandle,
                                   0,
                                   ACC_DATA_CHAR_SIZE,
                                   buff);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("AccData_Update failed: 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/* -------------------------------------------------------------------------*/

/**
 * @brief  Handle READ request on Char_A.
 *         Must call aci_gatt_allow_read() to release the GATT server.
 * @param  conn_handle  Current BLE connection handle
 * @param  x, y, z      Latest acceleration values to serve
 */
void AccData_ReadRequestCB(uint16_t conn_handle, int16_t x, int16_t y, int16_t z)
{
  /* Update value first so the client reads fresh data */
  AccData_Update(x, y, z);

  tBleStatus ret = aci_gatt_allow_read(conn_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("AccData_ReadRequestCB: aci_gatt_allow_read failed 0x%02X\r\n", ret);
  }
}