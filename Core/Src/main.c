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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define MAX_TASKS   4
#define STACK_SIZE  512   // 256 words = 1024 bytes per task

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

// ---- Test tasks ----
void Task_High(void)    // priority 0: wakes every 250 ms
{
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        count_high++;
        Task_Delay(250);
    }
}

void Task_Mid(void)     // priority 1: wakes every 1000 ms
{
    while (1) {
        count_mid++;
        Task_Delay(1000);
    }
}

char tx_buf[96];

void Task_Telemetry(void)   // priority 2: every 100 ms
{
    while (1) {
        int n = snprintf(tx_buf, sizeof(tx_buf),
                         "t=%lu ms  high=%lu  mid=%lu\r\n",
                         (unsigned long)sys_ticks,
                         (unsigned long)count_high,
                         (unsigned long)count_mid);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 50);
        Task_Delay(100);
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
  /* USER CODE BEGIN 2 */
  SysTick_Config(SystemCoreClock / 1000);   // SysTick fires every 1 ms
  Task_Init(0, Task_High, 0);
  Task_Init(1, Task_Mid,  1);
  Task_Init(2, Task_Telemetry,  2);
  Task_Init(3, Task_Idle, 3);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
