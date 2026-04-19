/**
  ******************************************************************************
  * @file    App/gatt_db.c
  * @brief   Custom Accelerometer GATT Service
  *          Char_A: Accel Data    (READ | NOTIFY)  6 bytes int16_t x3
  *          Char_B: Sampling Freq (READ | WRITE)   1 byte  enum index
  ******************************************************************************
  */

#include <string.h>
#include "gatt_db.h"
#include "bluenrg_def.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"

/* Private macros ------------------------------------------------------------*/
#define HOST_TO_LE_16(buf, val) \
  ( ((buf)[0] = (uint8_t)(val)), ((buf)[1] = (uint8_t)((val) >> 8)) )

/*
 * Fill a uint8_t[16] UUID array.
 * Arguments listed MSB first (index 15 down to index 0),
 * matching BlueNRG little-endian storage convention.
 */
#define COPY_UUID_128(uuid_struct, \
  b15,b14,b13,b12,b11,b10,b9,b8,b7,b6,b5,b4,b3,b2,b1,b0) \
do { \
  (uuid_struct)[0]=b0;  (uuid_struct)[1]=b1;  \
  (uuid_struct)[2]=b2;  (uuid_struct)[3]=b3;  \
  (uuid_struct)[4]=b4;  (uuid_struct)[5]=b5;  \
  (uuid_struct)[6]=b6;  (uuid_struct)[7]=b7;  \
  (uuid_struct)[8]=b8;  (uuid_struct)[9]=b9;  \
  (uuid_struct)[10]=b10;(uuid_struct)[11]=b11; \
  (uuid_struct)[12]=b12;(uuid_struct)[13]=b13; \
  (uuid_struct)[14]=b14;(uuid_struct)[15]=b15; \
} while(0)

/*
 * Service:  A0001000-74EE-43CE-B6A1-0002A5D5C51B
 * Char_A:   A0001001-74EE-43CE-B6A1-0002A5D5C51B
 * Char_B:   A0001002-74EE-43CE-B6A1-0002A5D5C51B
 */
#define COPY_ACC_SERVICE_UUID(u) \
  COPY_UUID_128(u,0xA0,0x00,0x10,0x00,0x74,0xEE,0x43,0xCE, \
                  0xB6,0xA1,0x00,0x02,0xA5,0xD5,0xC5,0x1B)

#define COPY_ACC_DATA_CHAR_UUID(u) \
  COPY_UUID_128(u,0xA0,0x00,0x10,0x01,0x74,0xEE,0x43,0xCE, \
                  0xB6,0xA1,0x00,0x02,0xA5,0xD5,0xC5,0x1B)

#define COPY_ACC_FREQ_CHAR_UUID(u) \
  COPY_UUID_128(u,0xA0,0x00,0x10,0x02,0x74,0xEE,0x43,0xCE, \
                  0xB6,0xA1,0x00,0x02,0xA5,0xD5,0xC5,0x1B)

#define ACC_SERVICE_MAX_ATTR_RECORDS  6U
#define ACC_DATA_CHAR_SIZE            6U
#define ACC_FREQ_CHAR_SIZE            1U
#define ACC_FREQ_DEFAULT_IDX          1U   /* 26 Hz */

/* Exported variables --------------------------------------------------------*/
uint16_t AccServiceHandle;
uint16_t AccDataCharHandle;
uint16_t AccFreqCharHandle;

/* =========================================================================
 * Add_AccService
 * ========================================================================= */
tBleStatus Add_AccService(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* ---- Service ---- */
  COPY_ACC_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,
                          uuid,
                          PRIMARY_SERVICE,
                          ACC_SERVICE_MAX_ATTR_RECORDS,
                          &AccServiceHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add_serv failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Char_A: READ | NOTIFY ---- */
  COPY_ACC_DATA_CHAR_UUID(uuid);
  ret = aci_gatt_add_char(AccServiceHandle,
                          UUID_TYPE_128,
                          uuid,
                          ACC_DATA_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0,
                          &AccDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add Char_A failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Char_B: READ | WRITE ---- */
  COPY_ACC_FREQ_CHAR_UUID(uuid);
  ret = aci_gatt_add_char(AccServiceHandle,
                          UUID_TYPE_128,
                          uuid,
                          ACC_FREQ_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          16, 0,
                          &AccFreqCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add Char_B failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* Set default Char_B value */
  uint8_t default_freq = ACC_FREQ_DEFAULT_IDX;
  ret = aci_gatt_update_char_value(AccServiceHandle, AccFreqCharHandle,
                                   0, ACC_FREQ_CHAR_SIZE, &default_freq);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: set default Char_B failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  PRINTF("Add_AccService: OK svc=0x%04X A=0x%04X B=0x%04X\r\n",
         AccServiceHandle, AccDataCharHandle, AccFreqCharHandle);
  return BLE_STATUS_SUCCESS;
}

/* =========================================================================
 * AccData_Update
 * ========================================================================= */
tBleStatus AccData_Update(int16_t x, int16_t y, int16_t z)
{
  tBleStatus ret;
  uint8_t buff[ACC_DATA_CHAR_SIZE];

  HOST_TO_LE_16(buff + 0, x);
  HOST_TO_LE_16(buff + 2, y);
  HOST_TO_LE_16(buff + 4, z);

  ret = aci_gatt_update_char_value(AccServiceHandle, AccDataCharHandle,
                                   0, ACC_DATA_CHAR_SIZE, buff);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("AccData_Update failed: 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/* =========================================================================
 * AccData_ReadRequestCB
 * ========================================================================= */
void AccData_ReadRequestCB(uint16_t conn_handle, int16_t x, int16_t y, int16_t z)
{
  AccData_Update(x, y, z);

  tBleStatus ret = aci_gatt_allow_read(conn_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gatt_allow_read failed 0x%02X\r\n", ret);
  }
}