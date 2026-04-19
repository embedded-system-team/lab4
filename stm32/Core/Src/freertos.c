/**
  ******************************************************************************
  * @file    Core/Src/freertos.c
  * @brief   FreeRTOS task implementations
  *
  *          TASK_BLE: BLE event loop
  *          TASK_ACC: LSM6DSL sampling and notification
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include "cmsis_os2.h"

#include "app_bluenrg_ms.h"
#include "gatt_db.h"
#include "b_l475e_iot01a1.h"
#include "b_l475e_iot01a1_bus.h"
#include "lsm6dsl.h"
#include "bluenrg_conf.h"   /* PRINTF */

/* Private defines -----------------------------------------------------------*/

#define LSM6DSL_I2C_ADDRESS   LSM6DSL_I2C_ADD_L
/*
 * ODR period lookup (ms):
 *   0 → 12.5 Hz →  80 ms
 *   1 →  26 Hz  →  38 ms
 *   2 →  52 Hz  →  19 ms
 *   3 → 104 Hz  →   9 ms
 */
static const uint32_t ODR_PERIOD_MS[4] = { 80U, 38U, 19U, 9U };
static const float    ODR_HZ[4]        = { 12.5f, 26.0f, 52.0f, 104.0f };

/* Private types -------------------------------------------------------------*/
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelData_t;

/* FreeRTOS object handles ---------------------------------------------------*/
osSemaphoreId_t    bleSemHandle;
osMutexId_t        freqMutexHandle;
osMessageQueueId_t accelQueueHandle;
osThreadId_t       taskBLEHandle;
osThreadId_t       taskACCHandle;

/* Shared state --------------------------------------------------------------*/
volatile uint8_t current_odr_idx = 1U;
volatile int16_t latest_acc_x    = 0;
volatile int16_t latest_acc_y    = 0;
volatile int16_t latest_acc_z    = 0;

/* LSM6DSL driver object -----------------------------------------------------*/
static LSM6DSL_Object_t lsm6dsl_obj;

/* Private function prototypes -----------------------------------------------*/
static void TaskBLE_Entry(void *argument);
static void TaskACC_Entry(void *argument);
static void ACC_Init(void);
static void ACC_SetODR(uint8_t odr_idx);

/* I2C bus binding -----------------------------------------------------------*/
static int32_t LSM6DSL_WriteReg_CB(uint16_t addr, uint16_t reg,
                                    uint8_t *data, uint16_t len)
{
  return BSP_I2C2_WriteReg(addr, reg, data, len);
}

static int32_t LSM6DSL_ReadReg_CB(uint16_t addr, uint16_t reg,
                                   uint8_t *data, uint16_t len)
{
  return BSP_I2C2_ReadReg(addr, reg, data, len);
}

/* =========================================================================
 * MX_FREERTOS_Init - called from main() before osKernelStart()
 * ========================================================================= */
void MX_FREERTOS_Init(void)
{
  bleSemHandle = osSemaphoreNew(1, 0, NULL);
  configASSERT(bleSemHandle != NULL);

  freqMutexHandle = osMutexNew(NULL);
  configASSERT(freqMutexHandle != NULL);

  accelQueueHandle = osMessageQueueNew(4, sizeof(AccelData_t), NULL);
  configASSERT(accelQueueHandle != NULL);

  const osThreadAttr_t taskBLE_attr = {
    .name       = "TASK_BLE",
    .stack_size = 1024 * 4,
    .priority   = osPriorityAboveNormal,
  };
  taskBLEHandle = osThreadNew(TaskBLE_Entry, NULL, &taskBLE_attr);
  configASSERT(taskBLEHandle != NULL);

  const osThreadAttr_t taskACC_attr = {
    .name       = "TASK_ACC",
    .stack_size = 256 * 4,
    .priority   = osPriorityNormal,
  };
  taskACCHandle = osThreadNew(TaskACC_Entry, NULL, &taskACC_attr);
  configASSERT(taskACCHandle != NULL);
}

/* =========================================================================
 * TASK_BLE
 * ========================================================================= */
static void TaskBLE_Entry(void *argument)
{
  UNUSED(argument);

  MX_BlueNRG_MS_Init();

  AccelData_t accel;

  for (;;)
  {
    osSemaphoreAcquire(bleSemHandle, 10);
    MX_BlueNRG_MS_Process();

    while (osMessageQueueGet(accelQueueHandle, &accel, NULL, 0) == osOK)
    {
      if (ble_connected)
      {
        AccData_Update(accel.x, accel.y, accel.z);
      }
    }
  }
}

/* =========================================================================
 * TASK_ACC
 * ========================================================================= */
static void TaskACC_Entry(void *argument)
{
  UNUSED(argument);

  ACC_Init();

  uint8_t     local_odr_idx  = 0xFF;
  uint32_t    period_ms;
  TickType_t  last_wake_time;
  LSM6DSL_Axes_t axes;
  AccelData_t    accel;

  last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    osMutexAcquire(freqMutexHandle, osWaitForever);
    uint8_t odr_idx = current_odr_idx;
    osMutexRelease(freqMutexHandle);

    if (odr_idx != local_odr_idx)
    {
      ACC_SetODR(odr_idx);
      local_odr_idx  = odr_idx;
      last_wake_time = xTaskGetTickCount();
    }

    period_ms = ODR_PERIOD_MS[local_odr_idx];

    if (LSM6DSL_ACC_GetAxes(&lsm6dsl_obj, &axes) == LSM6DSL_OK)
    {
      latest_acc_x = (int16_t)axes.x;
      latest_acc_y = (int16_t)axes.y;
      latest_acc_z = (int16_t)axes.z;

      accel.x = (int16_t)axes.x;
      accel.y = (int16_t)axes.y;
      accel.z = (int16_t)axes.z;

      osMessageQueuePut(accelQueueHandle, &accel, 0, 0);
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(period_ms));
  }
}

/* =========================================================================
 * LSM6DSL helpers
 * ========================================================================= */
static void ACC_Init(void)
{
  BSP_I2C2_Init();

  LSM6DSL_IO_t io_ctx = {
    .BusType  = LSM6DSL_I2C_BUS,
    .Address  = LSM6DSL_I2C_ADD_L,
    .Init     = BSP_I2C2_Init,
    .DeInit   = BSP_I2C2_DeInit,
    .ReadReg  = LSM6DSL_ReadReg_CB,
    .WriteReg = LSM6DSL_WriteReg_CB,
    .GetTick  = BSP_GetTick,
    .Delay    = HAL_Delay,
  };

  int32_t ret;

  ret = LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &io_ctx);
  ret = LSM6DSL_Init(&lsm6dsl_obj);

  if (ret != LSM6DSL_OK) { while(1); }

  LSM6DSL_ACC_Enable(&lsm6dsl_obj);
  LSM6DSL_ACC_SetFullScale(&lsm6dsl_obj, 2);
  ACC_SetODR(current_odr_idx);
  PRINTF("ACC_Init: LSM6DSL ready.\r\n");
}

static void ACC_SetODR(uint8_t odr_idx)
{
  if (odr_idx > 3U) odr_idx = 3U;
  int32_t ret = LSM6DSL_ACC_SetOutputDataRate(&lsm6dsl_obj, ODR_HZ[odr_idx]);
  if (ret != LSM6DSL_OK)
    PRINTF("ACC_SetODR: failed idx=%d\r\n", odr_idx);
  else {
    const uint16_t odr_int[] = {12, 26, 52, 104};
    PRINTF("ACC_SetODR: %d Hz\r\n", odr_int[odr_idx]);
  }
}

/* =========================================================================
 * BLE IRQ notify - called from hci_tl_lowlevel_isr()
 * ========================================================================= */
void BLE_IRQ_Notify(void)
{
  osSemaphoreRelease(bleSemHandle);
}