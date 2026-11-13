/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
   #include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define MAX_TASKS   4
#define STACK_SIZE  512   // 256 words = 2048 bytes per task

typedef enum {
    TASK_READY = 0,
    TASK_RUNNING,
    TASK_BLOCKED
} TaskState_t;

typedef struct {
    uint32_t   *stackPtr;
    uint32_t    priority;
    TaskState_t state;
    uint32_t    delayTicks;
} TCB_t;

TCB_t    TCB_Table[MAX_TASKS];
uint32_t Task_Stacks[MAX_TASKS][STACK_SIZE] __attribute__((aligned(8)));
uint8_t  current_task_id = 0;

volatile uint32_t sys_ticks = 0;          // incremented by our own SysTick
volatile uint8_t  scheduler_running = 0;
volatile uint32_t count_high = 0, count_mid = 0, count_low = 0;
volatile uint32_t echo_start = 0;
volatile uint32_t echo_width_cycles = 0;
volatile uint8_t  echo_done = 0;
volatile uint32_t distance_cm = 999;     // 999 = nothing in range
volatile uint8_t  obstacle_near = 0;
volatile uint32_t exti_count = 0;
volatile uint32_t deadline_misses = 0;
volatile uint32_t imu_count   = 0;
volatile uint32_t imu_min_us  = 0xFFFFFFFF;
volatile uint32_t imu_max_us  = 0;
volatile int32_t ax_mg = 0, ay_mg = 0, az_mg = 0;   // milli-g
volatile int32_t gz_dps_x10 = 0;                    // tenths of deg/s
volatile uint32_t imu_read_max_us = 0;              // slowest I2C read
#define SKID_ON_DPS_X10   300     // 30.0 deg/s
#define SKID_OFF_DPS_X10  200     // 20.0 deg/s
#define SKID_ON_SAMPLES   5       // 5 x 2 ms  = 10 ms
#define SKID_OFF_SAMPLES  25      // 25 x 2 ms = 50 ms

volatile int32_t  expected_yaw_x10 = 0;   // will come from the motor commands later
volatile uint8_t  skid_flag   = 0;
volatile uint32_t skid_events = 0;
volatile uint32_t i2c_errors = 0;
volatile uint32_t mpu_reinits = 0;
#define IMU_STALE_MS 20            // no good read for 20 ms = data is stale
volatile uint32_t imu_last_ok_tick = 0;
volatile uint8_t  imu_valid = 0;
int32_t ax_off = 0, ay_off = 0, gz_off = 0;   // raw-count offsets from calibration


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 72;            // 72 cycles per microsecond
    while ((DWT->CYCCNT - start) < ticks);
}
// Frees a bus where the sensor is holding SDA low, then restarts I2C1
static void i2c_bus_recover(void)
{
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;           // PB6 = SCL, PB7 = SDA
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    delay_us(10);
    for (int i = 0; i < 9; i++) {                // clock out a stuck slave
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); delay_us(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   delay_us(10);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); delay_us(10);   // STOP
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   delay_us(10);

    HAL_I2C_Init(&hi2c1);                        // restores the I2C pin setup
}

#define MPU_ADDR (0x68 << 1)
static int mpu_write_verify(uint8_t reg, uint8_t val)
{
    for (int i = 0; i < 5; i++) {
        uint8_t rb = 0xFF;
        if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10) == HAL_OK &&
            HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &rb, 1, 10) == HAL_OK &&
            rb == val)
            return 1;                      // written and read back correctly
        HAL_Delay(10);
    }
    return 0;
}

// returns 1 if every setting was written and verified
static int mpu_init(void)
{
    int ok = 1;
    ok &= mpu_write_verify(0x6B, 0x00);    // wake up
    HAL_Delay(100);
    ok &= mpu_write_verify(0x1A, 0x03);    // low-pass filter ~44 Hz
    ok &= mpu_write_verify(0x1B, 0x08);    // gyro  +-500 deg/s
    ok &= mpu_write_verify(0x1C, 0x08);    // accel +-4 g
    return ok;
}

static uint8_t i2c_fail_streak = 0;
static uint8_t i2c_skip = 0;

// Reads raw values. Returns 1 on success, 0 on I2C error.
static int mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gz)
{
    uint8_t r[14];

    if (i2c_skip) { i2c_skip--; return 0; }      // backing off, give others CPU time
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {   // bus stuck: don't wait 25 ms
        i2c_errors++;
        i2c_bus_recover();
        i2c_skip = 5;
        return 0;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, r, 14, 3) != HAL_OK) {
        i2c_errors++;
        if (++i2c_fail_streak >= 3) {
            i2c_bus_recover();
            i2c_fail_streak = 0;
            i2c_skip = 25;                       // pause about 50 ms
        }
        return 0;
    }
    i2c_fail_streak = 0;

    *ax = (int16_t)((r[0]  << 8) | r[1]);
    *ay = (int16_t)((r[2]  << 8) | r[3]);
    *az = (int16_t)((r[4]  << 8) | r[5]);
    *gz = (int16_t)((r[12] << 8) | r[13]);
    return 1;
}

static int mpu_reconfig(void)
{
    int ok = 1;
    ok &= mpu_write_verify(0x6B, 0x00);
    ok &= mpu_write_verify(0x1A, 0x03);
    ok &= mpu_write_verify(0x1B, 0x08);
    ok &= mpu_write_verify(0x1C, 0x08);
    return ok;
}

// If the sensor forgot its settings (power glitch), put them back.
static void mpu_check_config(void)
{
    uint8_t v = 0xFF;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &v, 1, 3) == HAL_OK
        && v != 0x08) {
        if (mpu_reconfig()) mpu_reinits++;
    }
}

// Board must be still. Averages 500 samples (~1 s).
static int mpu_calibrate(void)
{
    int32_t sax = 0, say = 0, sgz = 0;
    int good = 0;
    for (int i = 0; i < 500; i++) {
        int16_t ax, ay, az, gz;
        if (mpu_read_raw(&ax, &ay, &az, &gz)) {
            sax += ax; say += ay; sgz += gz; good++;
        }
        HAL_Delay(2);
    }
    if (good > 0) {
        ax_off = sax / good;
        ay_off = say / good;
        gz_off = sgz / good;
    }
    return good;
}
static void Task_Exit_Trap(void)
{
    while (1);   // a task should never return; if it does, we get stuck here
}

void Task_Init(uint8_t id, void (*taskFunc)(void), uint32_t priority)
{
    uint32_t *sp = &Task_Stacks[id][STACK_SIZE];   // one past the end

    // Hardware-saved frame
    *(--sp) = 0x01000000;               // xPSR (Thumb bit must be 1)
    *(--sp) = (uint32_t)taskFunc;       // PC: where the task starts
    *(--sp) = (uint32_t)Task_Exit_Trap; // LR: where it goes if it returns
    *(--sp) = 0x12121212;               // R12
    *(--sp) = 0x03030303;               // R3
    *(--sp) = 0x02020202;               // R2
    *(--sp) = 0x01010101;               // R1
    *(--sp) = 0x00000000;               // R0

    // Software-saved frame (pushed by our context switch later)
    *(--sp) = 0x11111111;               // R11
    *(--sp) = 0x10101010;               // R10
    *(--sp) = 0x09090909;               // R9
    *(--sp) = 0x08080808;               // R8
    *(--sp) = 0x07070707;               // R7
    *(--sp) = 0x06060606;               // R6
    *(--sp) = 0x05050505;               // R5
    *(--sp) = 0x04040404;               // R4

    TCB_Table[id].stackPtr   = sp;
    TCB_Table[id].priority   = priority;
    TCB_Table[id].state      = TASK_READY;
    TCB_Table[id].delayTicks = 0;
}

// Sleep the calling task for 'ticks' milliseconds
// Sleep the calling task for 'ticks' milliseconds
void Task_Delay(uint32_t ticks)
{
    __disable_irq();
    TCB_Table[current_task_id].delayTicks = ticks;
    TCB_Table[current_task_id].state      = TASK_BLOCKED;
    __enable_irq();

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;   // give up the CPU right now
    __DSB();
    __ISB();
}
// Sleep until (lastWake + period). Keeps a fixed rhythm with no drift.
// Sleep until (lastWake + period). Keeps a fixed rhythm with no drift.
void Task_DelayUntil(uint32_t *lastWake, uint32_t period)
{
    __disable_irq();
    uint32_t next = *lastWake + period;
    int32_t  wait = (int32_t)(next - sys_ticks);

    if (wait < 0) {                        // deadline missed
        deadline_misses++;
        *lastWake = sys_ticks + 1;         // resync
        wait = 1;                          // still yield 1 tick so others can run
    } else if (wait == 0) {                // exactly on time: keep going
        *lastWake = next;
        __enable_irq();
        return;
    } else {
        *lastWake = next;
    }

    TCB_Table[current_task_id].delayTicks = (uint32_t)wait;
    TCB_Table[current_task_id].state      = TASK_BLOCKED;
    __enable_irq();

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}
// Called every 1 ms from SysTick: wake sleepers, then ask for a re-evaluation
void Scheduler_Tick(void)
{
    if (!scheduler_running) return;

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (TCB_Table[i].state == TASK_BLOCKED && TCB_Table[i].delayTicks > 0) {
            if (--TCB_Table[i].delayTicks == 0)
                TCB_Table[i].state = TASK_READY;
        }
    }
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Picks the highest-priority READY task (lowest number = highest priority)
uint32_t *Scheduler_Switch(uint32_t *oldSp)
{
    __disable_irq();

    TCB_Table[current_task_id].stackPtr = oldSp;
    if (TCB_Table[current_task_id].state == TASK_RUNNING)
        TCB_Table[current_task_id].state = TASK_READY;

    uint8_t  best     = MAX_TASKS - 1;      // idle task is the fallback
    uint32_t bestPrio = 0xFFFFFFFF;
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (TCB_Table[i].state == TASK_READY && TCB_Table[i].priority < bestPrio) {
            bestPrio = TCB_Table[i].priority;
            best     = i;
        }
    }

    current_task_id = best;
    TCB_Table[best].state = TASK_RUNNING;
    uint32_t *sp = TCB_Table[best].stackPtr;

    __enable_irq();
    return sp;
}

void Task_IMU(void)   // priority 0: every 2 ms
{
    uint32_t last = sys_ticks;
    uint32_t prev = 0;
    uint8_t  started = 0;
    uint8_t  on_cnt = 0, off_cnt = 0;
    uint8_t chk=0;
    imu_last_ok_tick = sys_ticks;

    while (1) {
        uint32_t now = DWT->CYCCNT;
        if (started) {
            uint32_t period_us = (now - prev) / 72;
            if (period_us < imu_min_us) imu_min_us = period_us;
            if (period_us > imu_max_us) imu_max_us = period_us;
        }
        prev = now;
        started = 1;
        imu_count++;

        if (++chk >= 50) { chk = 0; mpu_check_config(); }   // every 100 ms

        int16_t ax, ay, az, gz;
        uint32_t t0 = DWT->CYCCNT;
        int ok = mpu_read_raw(&ax, &ay, &az, &gz);
        uint32_t dur_us = (DWT->CYCCNT - t0) / 72;
        if (dur_us > imu_read_max_us) imu_read_max_us = dur_us;

        if (ok) {
            imu_last_ok_tick = sys_ticks;
            ax_mg      = ((int32_t)ax - ax_off) * 1000 / 8192;
            ay_mg      = ((int32_t)ay - ay_off) * 1000 / 8192;
            az_mg      = (int32_t)az * 1000 / 8192;          // not calibrated: ~1000 when flat
            gz_dps_x10 = ((int32_t)gz - gz_off) * 100 / 655; // 65.5 LSB per deg/s
            int32_t diff = gz_dps_x10 - expected_yaw_x10;
            if (diff < 0) diff = -diff;


            if (!skid_flag) {
                if (diff > SKID_ON_DPS_X10) {
                    if (++on_cnt >= SKID_ON_SAMPLES) {
                        skid_flag = 1;
                        skid_events++;
                        off_cnt = 0;
                    }
                } else {
                    on_cnt = 0;
                }
            } else {
                if (diff < SKID_OFF_DPS_X10) {
                    if (++off_cnt >= SKID_OFF_SAMPLES) {
                        skid_flag = 0;
                        on_cnt = 0;
                    }
                } else {
                    off_cnt = 0;
                }
            }
        }
        imu_valid = ((sys_ticks - imu_last_ok_tick) <= IMU_STALE_MS);
        if (!imu_valid) { on_cnt = 0; off_cnt = 0; }   // don't count skid samples on stale data

        Task_DelayUntil(&last, 2);
    }
}
void Task_Ultrasonic(void)   // priority 1: every 20 ms
{
    uint32_t last = sys_ticks;
    while (1) {
        echo_done = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // 10 us trigger pulse
        delay_us(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

        Task_DelayUntil(&last, 20);                            // echo arrives meanwhile

        uint32_t width_us = echo_width_cycles / 72;
        if (echo_done && width_us < 25000)
            distance_cm = width_us / 58;
        else
            distance_cm = 999;                                 // nothing in range

        obstacle_near = (distance_cm < 20);
    }
}

char tx_buf[160];

void Task_Telemetry(void)   // priority 2: every 100 ms
{
    uint32_t last = sys_ticks;
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // heartbeat

        int n = snprintf(tx_buf, sizeof(tx_buf),
                         "t=%lu dist=%lu imu=%lu jit[%lu..%lu] rd=%luus gz=%ld ay=%ld az=%ld err=%lu  miss=%lu skid=%u ev=%lu rst=%lu v=%u\r\n",
                         (unsigned long)sys_ticks,
                         (unsigned long)distance_cm,
                         (unsigned long)imu_count,
                         (unsigned long)imu_min_us,
                         (unsigned long)imu_max_us,
                         (unsigned long)imu_read_max_us,
                         (long)gz_dps_x10,
                         (long)ay_mg,
                         (long)az_mg,
                         (unsigned long)i2c_errors,
                         (unsigned long)deadline_misses,
                         (unsigned)skid_flag,
                         (unsigned long)skid_events,
                         (unsigned long)mpu_reinits,
                         (unsigned)imu_valid);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 50);
        Task_DelayUntil(&last, 100);
    }
}
void Task_Idle(void)    // priority 3: runs only if nothing else can
{
    while (1) { }
}
// The context switch itself
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "mrs   r0, psp            \n"  // r0 = running task's stack pointer
        "stmdb r0!, {r4-r11}      \n"  // save R4-R11 onto that stack
        "push  {r3, lr}           \n"  // keep EXC_RETURN safe (r3 keeps alignment)
        "bl    Scheduler_Switch   \n"  // r0 = old sp in, new task's sp out
        "pop   {r3, lr}           \n"
        "ldmia r0!, {r4-r11}      \n"  // restore the next task's R4-R11
        "msr   psp, r0            \n"  // PSP now points at its hardware frame
        "bx    lr                 \n"  // hardware pops the rest and jumps to the task
    );
}

// Starts the very first task (no context to save yet)
__attribute__((naked)) void Start_First_Task(uint32_t *stackTop, uint32_t entry)
{
    __asm volatile (
        "msr  psp, r0      \n"   // task stack
        "movs r2, #2       \n"
        "msr  control, r2  \n"   // thread mode now uses PSP
        "isb               \n"
        "bx   r1           \n"   // jump into the task
    );
}

void Scheduler_Start(void)
{
    NVIC_SetPriority(PendSV_IRQn, 15);   // lowest: never blocks a real interrupt
    NVIC_SetPriority(SysTick_IRQn, 14);

    current_task_id = 0;
    TCB_Table[0].state = TASK_RUNNING;
    scheduler_running = 1;

    uint32_t *sp = TCB_Table[0].stackPtr;
    Start_First_Task(sp + 16, sp[14]);   // skip the 16-word fake frame; sp[14] = task's PC
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_Delay(300);
    int init_ok = 0;
    for (int tries = 0; tries < 5 && !init_ok; tries++) {
        if (tries) i2c_bus_recover();    // unstick the bus before retrying
        init_ok = mpu_init();
    }
    HAL_Delay(500);                      // keep the board still
    int good = 0;
    for (int tries = 0; tries < 3 && good < 450; tries++)
        good = mpu_calibrate();
    {
        char b[120];
        int n = snprintf(b, sizeof(b),
                         "CAL init=%d good=%d ax_off=%ld ay_off=%ld gz_off=%ld errs=%lu\r\n",
                         init_ok, good, (long)ax_off, (long)ay_off, (long)gz_off,
                         (unsigned long)i2c_errors);
        HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 100);
    }
    i2c_errors = 0;                      // from here on, count only runtime errors

  SysTick_Config(SystemCoreClock / 1000);
  Task_Init(0, Task_IMU,       0);
  Task_Init(1, Task_Ultrasonic, 1);
  Task_Init(2, Task_Telemetry,  2);
  Task_Init(3, Task_Idle,       3);
  Scheduler_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) {
    	   exti_count++;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
            echo_start = DWT->CYCCNT;                          // rising edge
        else {
            echo_width_cycles = DWT->CYCCNT - echo_start;      // falling edge
            echo_done = 1;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
