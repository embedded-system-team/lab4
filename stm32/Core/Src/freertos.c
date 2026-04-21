/**
  ******************************************************************************
  * @file    Core/Src/freertos.c
  * @brief   FreeRTOS task implementations
  *
  *  TASK_BLE: BLE event loop + motion event notification
  *  TASK_ACC: LSM6DSL sampling, significant motion detection init
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "app_bluenrg_ms.h"
#include "gatt_db.h"
#include "b_l475e_iot01a1.h"
#include "b_l475e_iot01a1_bus.h"
#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"
#include "bluenrg_conf.h"   /* PRINTF */
#include "main.h"           /* LSM6DSL_INT1_EXTI11_Pin, _GPIO_Port */

/* Private defines -----------------------------------------------------------*/
#define LSM6DSL_I2C_ADDRESS   LSM6DSL_I2C_ADD_L   /* SA0 = GND → 0xD5 */

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
osSemaphoreId_t    motionSemHandle;   /* given by EXTI11 ISR */
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
static void ACC_Motion_Init(void);

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
 * MX_FREERTOS_Init
 * ========================================================================= */
void MX_FREERTOS_Init(void)
{
  bleSemHandle = osSemaphoreNew(1, 0, NULL);
  configASSERT(bleSemHandle != NULL);

  motionSemHandle = osSemaphoreNew(1, 0, NULL);
  configASSERT(motionSemHandle != NULL);

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
    /* Wait for BLE IRQ or timeout */
    osSemaphoreAcquire(bleSemHandle, 10);
    MX_BlueNRG_MS_Process();

    /* Drain accel queue → BLE notification */
    while (osMessageQueueGet(accelQueueHandle, &accel, NULL, 0) == osOK)
    {
      if (ble_connected)
      {
        AccData_Update(accel.x, accel.y, accel.z);
  }
    }

    /* Check for significant motion event from EXTI11 ISR */
    if (osSemaphoreAcquire(motionSemHandle, 0) == osOK)
    {
      if (ble_connected)
      {
        Motion_Notify();
      }
      lsm6dsl_motion_sens_set(&lsm6dsl_obj.Ctx, PROPERTY_ENABLE);
    }
  }
}

/* ======================================================================
 * TASK_ACC
 * ========================================================================= */
static void TaskACC_Entry(void *argument)
{
  UNUSED(argument);

  ACC_Init();
  ACC_Motion_Init();

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
    .Address  = LSM6DSL_I2C_ADDRESS,
    .Init     = BSP_I2C2_Init,
    .DeInit   = BSP_I2C2_DeInit,
    .ReadReg  = LSM6DSL_ReadReg_CB,
    .WriteReg = LSM6DSL_WriteReg_CB,
    .GetTick  = BSP_GetTick,
    .Delay    = HAL_Delay,
  };

  int32_t ret;

  ret = LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &io_ctx);
  if (ret != LSM6DSL_OK) { PRINTF("ACC_Init: RegisterBusIO failed\r\n"); while(1); }

  ret = LSM6DSL_Init(&lsm6dsl_obj);
  if (ret != LSM6DSL_OK) { PRINTF("ACC_Init: Init failed\r\n"); while(1); }

  ret = LSM6DSL_ACC_Enable(&lsm6dsl_obj);
  if (ret != LSM6DSL_OK) { PRINTF("ACC_Init: Enable failed\r\n"); while(1); }

  LSM6DSL_ACC_SetFullScale(&lsm6dsl_obj, 16);
  ACC_SetODR(current_odr_idx);

  PRINTF("ACC_Init: LSM6DSL ready.\r\n");
}

static void ACC_SetODR(uint8_t odr_idx)
{
  if (odr_idx > 3U) odr_idx = 3U;
  const uint16_t odr_int[] = {12, 26, 52, 104};
  int32_t ret = LSM6DSL_ACC_SetOutputDataRate(&lsm6dsl_obj, ODR_HZ[odr_idx]);
  if (ret != LSM6DSL_OK)
    PRINTF("ACC_SetODR: failed idx=%d\r\n", odr_idx);
  else
    PRINTF("ACC_SetODR: %d Hz\r\n", odr_int[odr_idx]);
}

/**
 * @brief  Enable LSM6DSL significant motion detection on INT1 (PD11).
 *
 *         Significant motion uses the embedded pedometer engine.
 *         Steps:
 *         1. Set ODR to 26 Hz (required by significant motion engine)
 *         2. Enable significant motion detection
 *         3. Route INT1 to significant motion interrupt
 *         4. Configure PD11 EXTI (rising edge, priority 5)
 */
static void ACC_Motion_Init(void)
{
  /* Set significant motion threshold
  * Range: 0x01–0x3F (steps)
  * Default: 0x06 (6 steps)
  * Lower = more sensitive
  */
  uint8_t threshold = 0x06;
  if (lsm6dsl_motion_threshold_set(&lsm6dsl_obj.Ctx, &threshold) != 0)
  {
  PRINTF("ACC_Motion_Init: threshold_set failed\r\n");
}
  /* Significant motion requires ODR = 26 Hz on the accelerometer.
   * It shares the pedometer engine, so we set ODR via the low-level
   * reg API to avoid overriding the user-set ODR on the high-level driver.
   * The interrupt fires independently of the sampling ODR. */

  /* Enable significant motion detection */
  if (lsm6dsl_motion_sens_set(&lsm6dsl_obj.Ctx, PROPERTY_ENABLE) != 0)
  {
    PRINTF("ACC_Motion_Init: motion_sens_set failed\r\n");
    return;
  }

  /* Route significant motion interrupt to INT1 pin */
  lsm6dsl_int1_route_t int1_route = {0};
  if (lsm6dsl_pin_int1_route_get(&lsm6dsl_obj.Ctx, &int1_route) != 0)
  {
    PRINTF("ACC_Motion_Init: int1_route_get failed\r\n");
    return;
  }

  int1_route.int1_sign_mot = 1;  /* route significant motion to INT1 */

  if (lsm6dsl_pin_int1_route_set(&lsm6dsl_obj.Ctx, int1_route) != 0)
  {
    PRINTF("ACC_Motion_Init: int1_route_set failed\r\n");
    return;
  }

  /* Configure PD11 (LSM6DSL_INT1_EXTI11) as EXTI input, rising edge */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin  = LSM6DSL_INT1_EXTI11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct);

  /* Priority must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
   * to safely call osSemaphoreRelease from ISR */
  HAL_NVIC_SetPriority(LSM6DSL_INT1_EXTI11_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LSM6DSL_INT1_EXTI11_EXTI_IRQn);

  PRINTF("ACC_Motion_Init: significant motion detection enabled on INT1 (PD11).\r\n");
}

/* =========================================================================
 * BLE IRQ notify - called from hci_tl_lowlevel_isr() (EXTI6 ISR context)
 * ========================================================================= */
void BLE_IRQ_Notify(void)
{
  osSemaphoreRelease(bleSemHandle);
}

/* =========================================================================
 * Motion IRQ notify - called from EXTI15_10_IRQHandler (PD11 / EXTI11)
 * ========================================================================= */
void Motion_IRQ_Notify(void)
{
  osSemaphoreRelease(motionSemHandle);
}