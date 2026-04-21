/**
  ******************************************************************************
  * @file    App/gatt_db.c
  * @brief   Custom Accelerometer GATT Service
  *
  *  Char_A: Accel Data     (READ | NOTIFY)  6 bytes int16_t x3, unit: mg
  *  Char_B: Sampling Freq  (READ | WRITE)   1 byte  enum index 0-3
  *  Char_C: Motion Event   (NOTIFY)         1 byte  0x01 = significant motion
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
 * Char_A:   A0001001-74EE-43CE-B6A1-0002A5D5C51B  Accel Data
 * Char_B:   A0001002-74EE-43CE-B6A1-0002A5D5C51B  Sampling Freq
 * Char_C:   A0001003-74EE-43CE-B6A1-0002A5D5C51B  Motion Event
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

#define COPY_MOTION_CHAR_UUID(u) \
  COPY_UUID_128(u,0xA0,0x00,0x10,0x03,0x74,0xEE,0x43,0xCE, \
                  0xB6,0xA1,0x00,0x02,0xA5,0xD5,0xC5,0x1B)

/*
 * Attribute count:
 *   1  service declaration
 *   3  Char_A: declaration + value + CCCD (NOTIFY)
 *   2  Char_B: declaration + value
 *   3  Char_C: declaration + value + CCCD (NOTIFY)
 *   = 9 → use 10 for safety margin
 */
#define ACC_SERVICE_MAX_ATTR_RECORDS  10U
#define ACC_DATA_CHAR_SIZE             6U
#define ACC_FREQ_CHAR_SIZE             1U
#define ACC_MOTION_CHAR_SIZE           1U
#define ACC_FREQ_DEFAULT_IDX           1U   /* 26 Hz */

/* Exported variables --------------------------------------------------------*/
uint16_t AccServiceHandle;
uint16_t AccDataCharHandle;
uint16_t AccFreqCharHandle;
uint16_t MotionCharHandle;

/* =========================================================================
 * Add_AccService
 * ========================================================================= */
tBleStatus Add_AccService(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* ---- Service ---- */
  COPY_ACC_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE,
                          ACC_SERVICE_MAX_ATTR_RECORDS, &AccServiceHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add_serv failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Char_A: Accel Data READ | NOTIFY ---- */
  COPY_ACC_DATA_CHAR_UUID(uuid);
  ret = aci_gatt_add_char(AccServiceHandle, UUID_TYPE_128, uuid,
                          ACC_DATA_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &AccDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add Char_A failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  /* ---- Char_B: Sampling Freq READ | WRITE ---- */
  COPY_ACC_FREQ_CHAR_UUID(uuid);
  ret = aci_gatt_add_char(AccServiceHandle, UUID_TYPE_128, uuid,
                          ACC_FREQ_CHAR_SIZE,
                          CHAR_PROP_READ | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          16, 0, &AccFreqCharHandle);
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

  /* ---- Char_C: Motion Event NOTIFY only ---- */
  COPY_MOTION_CHAR_UUID(uuid);
  ret = aci_gatt_add_char(AccServiceHandle, UUID_TYPE_128, uuid,
                          ACC_MOTION_CHAR_SIZE,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          16, 0, &MotionCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Add_AccService: add Char_C failed 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  PRINTF("Add_AccService: OK svc=0x%04X A=0x%04X B=0x%04X C=0x%04X\r\n",
         AccServiceHandle, AccDataCharHandle, AccFreqCharHandle, MotionCharHandle);
  return BLE_STATUS_SUCCESS;
}

/* =========================================================================
 * AccData_Update - update Char_A and send notification
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
 * AccData_ReadRequestCB - handle READ request on Char_A
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

/* =========================================================================
 * Motion_Notify - send significant motion event notification via Char_C
 * ========================================================================= */
tBleStatus Motion_Notify(void)
{
  uint8_t payload = 0x01;  /* 0x01 = significant motion detected */

  tBleStatus ret = aci_gatt_update_char_value(AccServiceHandle,
                                              MotionCharHandle,
                                              0, ACC_MOTION_CHAR_SIZE,
                                              &payload);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Motion_Notify failed: 0x%02X\r\n", ret);
    return BLE_STATUS_ERROR;
  }

  PRINTF("Motion_Notify: significant motion event sent.\r\n");
  return BLE_STATUS_SUCCESS;
}