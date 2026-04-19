/**
  ******************************************************************************
  * @file    Core/Src/freertos.c
  * @brief   FreeRTOS task implementations
  *
  *          TASK_BLE:
  *            - Waits for BLE IRQ semaphore (given by hci_tl_lowlevel_isr)
  *            - Calls MX_BlueNRG_MS_Process() to dispatch HCI events
  *
  *          TASK_ACC:
  *            - Reads LSM6DSL acceleration via official BSP driver
  *            - Sends data to TASK_BLE via accelQueue
  *            - Adjusts sampling period according to current_odr_idx
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"

#include "app_bluenrg_ms.h"
#include "gatt_db.h"
#include "b_l475e_iot01a1.h"

/* LSM6DSL official BSP driver */
#include "b_l475e_iot01a1_motion_sensors.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/

/*
 * ODR period lookup table (milliseconds)
 * Index: AccODR_Index_t
 *   0 → 12.5 Hz →  80 ms
 *   1 →  26 Hz  →  38 ms
 *   2 →  52 Hz  →  19 ms
 *   3 → 104 Hz  →   9 ms
 */
static const uint32_t ODR_PERIOD_MS[4] = { 80U, 38U, 19U, 9U };

/*
 * ODR lookup table for BSP driver
 * Maps AccODR_Index_t to BSP_MOTION_SENSOR_SetOutputDataRate() parameter (Hz)
 */
static const float ODR_HZ[4] = { 12.5f, 26.0f, 52.0f, 104.0f };

/* Private variables ---------------------------------------------------------*/

/* Acceleration data packet passed through accelQueue */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelData_t;

/* ---- FreeRTOS object handles (defined here, declared extern elsewhere) ---- */

/* Semaphore: given by hci_tl_lowlevel_isr(), taken by TASK_BLE */
osSemaphoreId_t bleSemHandle;

/* Mutex: protects current_odr_idx between TASK_BLE (write) and TASK_ACC (read) */
osMutexId_t freqMutexHandle;

/* Queue: TASK_ACC → TASK_BLE, carries AccelData_t */
osMessageQueueId_t accelQueueHandle;

/* Task handles */
osThreadId_t taskBLEHandle;
osThreadId_t taskACCHandle;

/* Shared state -----------------------------------------------------------*/

/* Current ODR index - written by TASK_BLE on Char_B write, read by TASK_ACC */
volatile uint8_t current_odr_idx = 1U;   /* default: 26 Hz */

/* Latest acceleration values - written by TASK_ACC, read by TASK_BLE on READ req */
volatile int16_t latest_acc_x = 0;
volatile int16_t latest_acc_y = 0;
volatile int16_t latest_acc_z = 0;

/* Private function prototypes -----------------------------------------------*/
static void TaskBLE_Entry(void *argument);
static void TaskACC_Entry(void *argument);
static void ACC_Init(void);
static void ACC_SetODR(uint8_t odr_idx);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* =========================================================================
 * MX_FREERTOS_Init
 * Called from main() before osKernelStart()
 * ========================================================================= */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Create semaphore for BLE IRQ signalling */
  bleSemHandle = osSemaphoreNew(1, 0, NULL);
  configASSERT(bleSemHandle != NULL);

  /* Create mutex for ODR index protection */
  freqMutexHandle = osMutexNew(NULL);
  configASSERT(freqMutexHandle != NULL);

  /* Create queue: depth=4, item=sizeof(AccelData_t) */
  accelQueueHandle = osMessageQueueNew(4, sizeof(AccelData_t), NULL);
  configASSERT(accelQueueHandle != NULL);

  /* Create TASK_BLE */
  const osThreadAttr_t taskBLE_attr = {
    .name       = "TASK_BLE",
    .stack_size = 512 * 4,   /* 512 words */
    .priority   = osPriorityAboveNormal,
  };
  taskBLEHandle = osThreadNew(TaskBLE_Entry, NULL, &taskBLE_attr);
  configASSERT(taskBLEHandle != NULL);

  /* Create TASK_ACC */
  const osThreadAttr_t taskACC_attr = {
    .name       = "TASK_ACC",
    .stack_size = 256 * 4,   /* 256 words */
    .priority   = osPriorityNormal,
  };
  taskACCHandle = osThreadNew(TaskACC_Entry, NULL, &taskACC_attr);
  configASSERT(taskACCHandle != NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
}

/* =========================================================================
 * TASK_BLE
 * ========================================================================= */

/**
 * @brief  TASK_BLE entry function.
 *
 *         Initialization sequence:
 *           1. Initialize BLE stack and custom GATT service
 *
 *         Event loop:
 *           2. Wait for BLE IRQ semaphore (timeout 10 ms for queue polling)
 *           3. Call MX_BlueNRG_MS_Process() to dispatch HCI events
 *           4. Drain accelQueue and send BLE notifications if connected
 */
static void TaskBLE_Entry(void *argument)
{
  UNUSED(argument);

  /* Step 1: BLE stack initialization */
  MX_BlueNRG_MS_Init();

  AccelData_t accel;

  for (;;)
  {
    /*
     * Step 2: Wait for BLE IRQ semaphore.
     * Timeout = 10 ms so we can still drain the accelQueue
     * even if no BLE IRQ arrives.
     */
    osSemaphoreAcquire(bleSemHandle, 10);

    /* Step 3: Process pending HCI events */
    MX_BlueNRG_MS_Process();

    /* Step 4: Send queued acceleration data as BLE notifications */
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

/**
 * @brief  TASK_ACC entry function.
 *
 *         Initialization sequence:
 *           1. Initialize LSM6DSL accelerometer
 *
 *         Sampling loop:
 *           2. Read current ODR index (mutex protected)
 *           3. Update LSM6DSL ODR if changed
 *           4. Read XYZ acceleration from LSM6DSL
 *           5. Update latest_acc_xyz (for READ requests)
 *           6. Send data to TASK_BLE via accelQueue
 *           7. Delay for the appropriate period
 */
static void TaskACC_Entry(void *argument)
{
  UNUSED(argument);

  /* Step 1: Initialize LSM6DSL */
  ACC_Init();

  uint8_t  local_odr_idx  = 0xFF;  /* invalid sentinel to force first update */
  uint32_t period_ms;
  TickType_t last_wake_time;
  BSP_MOTION_SENSOR_Axes_t axes;
  AccelData_t accel;

  last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    /* Step 2: Read current ODR index */
    osMutexAcquire(freqMutexHandle, osWaitForever);
    uint8_t odr_idx = current_odr_idx;
    osMutexRelease(freqMutexHandle);

    /* Step 3: Update LSM6DSL ODR only if changed */
    if (odr_idx != local_odr_idx)
    {
      ACC_SetODR(odr_idx);
      local_odr_idx = odr_idx;
      /* Reset timing reference after ODR change */
      last_wake_time = xTaskGetTickCount();
    }

    period_ms = ODR_PERIOD_MS[local_odr_idx];

    /* Step 4: Read acceleration from LSM6DSL */
    if (BSP_MOTION_SENSOR_GetAxes(0, MOTION_ACCELERO, &axes) == BSP_ERROR_NONE)
    {
      /* Step 5: Update latest values for READ requests */
      latest_acc_x = (int16_t)axes.xval;
      latest_acc_y = (int16_t)axes.yval;
      latest_acc_z = (int16_t)axes.zval;

      /* Step 6: Enqueue for TASK_BLE notification */
      accel.x = (int16_t)axes.xval;
      accel.y = (int16_t)axes.yval;
      accel.z = (int16_t)axes.zval;

      /*
       * osMessageQueuePut with timeout=0: if queue is full, drop the sample.
       * This prevents TASK_ACC from blocking when BLE is slow.
       */
      osMessageQueuePut(accelQueueHandle, &accel, 0, 0);
    }

    /* Step 7: Delay until next sample period */
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(period_ms));
  }
}

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief  Initialize LSM6DSL in accelerometer-only mode.
 *         Uses official X-CUBE-MEMS1 BSP driver.
 */
static void ACC_Init(void)
{
  int32_t ret;

  ret = BSP_MOTION_SENSOR_Init(0, MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    PRINTF("ACC_Init: BSP_MOTION_SENSOR_Init failed %ld\r\n", ret);
    while (1);
  }

  /* Enable accelerometer */
  ret = BSP_MOTION_SENSOR_Enable(0, MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    PRINTF("ACC_Init: BSP_MOTION_SENSOR_Enable failed %ld\r\n", ret);
    while (1);
  }

  /* Set initial ODR */
  ACC_SetODR(current_odr_idx);

  /* Set full-scale to ±2g (default, gives best resolution) */
  ret = BSP_MOTION_SENSOR_SetFullScale(0, MOTION_ACCELERO, 2);
  if (ret != BSP_ERROR_NONE)
  {
    PRINTF("ACC_Init: SetFullScale failed %ld\r\n", ret);
  }

  PRINTF("ACC_Init: LSM6DSL initialized.\r\n");
}

/**
 * @brief  Set LSM6DSL accelerometer ODR via BSP driver.
 * @param  odr_idx  AccODR_Index_t value (0–3)
 */
static void ACC_SetODR(uint8_t odr_idx)
{
  if (odr_idx > ACC_ODR_MAX)
  {
    odr_idx = ACC_ODR_MAX;
  }

  int32_t ret = BSP_MOTION_SENSOR_SetOutputDataRate(0,
                                                     MOTION_ACCELERO,
                                                     ODR_HZ[odr_idx]);
  if (ret != BSP_ERROR_NONE)
  {
    PRINTF("ACC_SetODR: failed for idx=%d (%.1f Hz), ret=%ld\r\n",
           odr_idx, (double)ODR_HZ[odr_idx], ret);
  }
  else
  {
    PRINTF("ACC_SetODR: ODR set to %.1f Hz\r\n", (double)ODR_HZ[odr_idx]);
  }
}

/* =========================================================================
 * BLE IRQ semaphore give - called from hci_tl_lowlevel_isr()
 * ========================================================================= */

/**
 * @brief  Called from hci_tl_lowlevel_isr() (EXTI6 ISR context).
 *         Gives the BLE semaphore to wake TASK_BLE.
 *
 *         NOTE: Replace the hci_notify_asynch_evt() call inside
 *         hci_tl_lowlevel_isr() with this function call.
 */
void BLE_IRQ_Notify(void)
{
  osSemaphoreRelease(bleSemHandle);
}