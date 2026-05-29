/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TYPE_MANUAL  0
#define TYPE_AUTO    1

#define CMD_UP     (1 << 0)
#define CMD_DOWN   (1 << 1)
#define CMD_LEFT   (1 << 2)
#define CMD_RIGHT  (1 << 3)
#define CMD_TOGGLE (1 << 4)

typedef struct {
    uint8_t  msg_type;
    uint16_t manual_cmd;
    int16_t  error_x;
    int16_t  error_y;
} TurretMsg_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

// UART 수신 버퍼
uint8_t rx_byte;
char    rx_buffer[30];
uint8_t rx_index = 0;

// ISR → Task 데이터 전달용 변수
// ISR에서는 플래그만 세우고, sscanf는 MotorTask에서 처리
volatile uint8_t uart_msg_ready = 0;   // 완성된 문장이 있으면 1
char             uart_parse_buf[30];   // ISR이 복사해 둔 완성 문자열
/* USER CODE END Variables */

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeypadTask */
osThreadId_t KeypadTaskHandle;
const osThreadAttr_t KeypadTask_attributes = {
  .name = "KeypadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KeyQueue */
osMessageQueueId_t KeyQueueHandle;
const osMessageQueueAttr_t KeyQueue_attributes = {
  .name = "KeyQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint16_t Scan_Keypad(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  KeyQueueHandle = osMessageQueueNew(16, sizeof(TurretMsg_t), &KeyQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  MotorTaskHandle  = osThreadNew(StartDefaultTask, NULL, &MotorTask_attributes);
  KeypadTaskHandle = osThreadNew(StartTask02,      NULL, &KeypadTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  MotorTask: 서보 제어 + UART 파싱 처리
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TurretMsg_t rx_msg;

  uint16_t pulse_x = 1500;
  uint16_t pulse_y = 1500;

  const uint16_t step_size = 16;

  // ── 제어 상수 ────────────────────────────────────────────
  const float Kp_x      = 0.08f;
  const float Kp_y      = 0.08f;
  const float Kd_x      = 0.25f;   // D항: 브레이크 강도
  const float Kd_y      = 0.25f;   // 진동 있으면 올리고, 굳으면 낮추기
  const int   max_speed = 15;
  const int   deadzone  = 10;      // 서보 버징 방지
  const float max_pulse = 2300.0f;
  const float min_pulse =  700.0f;
  // ─────────────────────────────────────────────────────────

  // D항 계산용 이전 오차 + float 펄스 누적
  float prev_error_x = 0.0f;
  float prev_error_y = 0.0f;
  float pulse_x_f    = 1500.0f;
  float pulse_y_f    = 1500.0f;

  uint8_t current_mode = TYPE_MANUAL;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  TIM3->CCR1 = pulse_x;
  TIM3->CCR2 = pulse_y;

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  for(;;)
  {
    // ── UART 파싱 (ISR에서 sscanf 금지 → 태스크에서 처리) ──
    if(uart_msg_ready)
    {
        uart_msg_ready = 0;

        int err_x = 0, err_y = 0;
        if(sscanf(uart_parse_buf, "%d,%d", &err_x, &err_y) == 2)
        {
            TurretMsg_t tx_msg;
            tx_msg.msg_type   = TYPE_AUTO;
            tx_msg.manual_cmd = 0;
            tx_msg.error_x    = (int16_t)err_x;
            tx_msg.error_y    = (int16_t)err_y;
            osMessageQueuePut(KeyQueueHandle, &tx_msg, 0, 0);
        }
    }

    if(osMessageQueueGet(KeyQueueHandle, &rx_msg, NULL, 10) == osOK)
    {
        // ── 모드 전환 (TOGGLE 버튼) ──────────────────────────
        if(rx_msg.msg_type == TYPE_MANUAL && (rx_msg.manual_cmd & CMD_TOGGLE))
        {
            current_mode = (current_mode == TYPE_AUTO) ? TYPE_MANUAL : TYPE_AUTO;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        }

        // ── 수동 모드 ─────────────────────────────────────────
        if(current_mode == TYPE_MANUAL)
        {
            if(rx_msg.msg_type == TYPE_MANUAL)
            {
                if(rx_msg.manual_cmd & CMD_LEFT)  pulse_x -= step_size;
                if(rx_msg.manual_cmd & CMD_RIGHT) pulse_x += step_size;
                if(rx_msg.manual_cmd & CMD_UP)    pulse_y -= step_size;
                if(rx_msg.manual_cmd & CMD_DOWN)  pulse_y += step_size;

                // [FIX] 수동 소프트 리미트 + CCR 반영
                if(pulse_x > (uint16_t)max_pulse) pulse_x = (uint16_t)max_pulse;
                if(pulse_x < (uint16_t)min_pulse) pulse_x = (uint16_t)min_pulse;
                if(pulse_y > (uint16_t)max_pulse) pulse_y = (uint16_t)max_pulse;
                if(pulse_y < (uint16_t)min_pulse) pulse_y = (uint16_t)min_pulse;

                TIM3->CCR1 = pulse_x;
                TIM3->CCR2 = pulse_y;
            }
        }

        // ── 자동 모드 (PD 제어) ───────────────────────────────
        else if(current_mode == TYPE_AUTO)
        {
            if(rx_msg.msg_type == TYPE_AUTO)
            {
                float ex = (float)rx_msg.error_x;
                float ey = (float)rx_msg.error_y;

                // [FIX 버그2/3] ex/ey 복사 후 데드존 처리 (중복 제거)
                //              prev_error 비교도 데드존 적용된 값으로 일관성 유지
                if(ex > -(float)deadzone && ex < (float)deadzone) ex = 0.0f;
                if(ey > -(float)deadzone && ey < (float)deadzone) ey = 0.0f;

                // LED 락온 판정 (나중에 레이저 포인터로 변경 예정)
                if(ex == 0.0f && ey == 0.0f)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                else
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

                // ── PD 제어 ────────────────────────────────────
                // P항: 오차에 비례 (멀수록 빠름)
                // D항: 오차 변화율 (줄어드는 중이면 자동 감속 → 오버슈트 억제)
                float delta_x = Kp_x * ex + Kd_x * (ex - prev_error_x);
                float delta_y = Kp_y * ey + Kd_y * (ey - prev_error_y);

                prev_error_x = ex;
                prev_error_y = ey;

                // 스피드 리미터
                if(delta_x >  (float)max_speed) delta_x =  (float)max_speed;
                if(delta_x < -(float)max_speed) delta_x = -(float)max_speed;
                if(delta_y >  (float)max_speed) delta_y =  (float)max_speed;
                if(delta_y < -(float)max_speed) delta_y = -(float)max_speed;

                // float 누적
                pulse_x_f += delta_x;
                pulse_y_f -= delta_y;

                // [FIX 버그1] 소프트 리미트를 pulse_x_f 기준으로 통일
                if(pulse_x_f > max_pulse) pulse_x_f = max_pulse;
                if(pulse_x_f < min_pulse) pulse_x_f = min_pulse;
                if(pulse_y_f > max_pulse) pulse_y_f = max_pulse;
                if(pulse_y_f < min_pulse) pulse_y_f = min_pulse;

                TIM3->CCR1 = (uint16_t)pulse_x_f;
                TIM3->CCR2 = (uint16_t)pulse_y_f;
            }
            else
            {
                // 물체 없음 → 중앙 대기 + 전체 리셋
                pulse_x_f    = 1500.0f;
                pulse_y_f    = 1500.0f;
                prev_error_x = 0.0f;
                prev_error_y = 0.0f;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

                TIM3->CCR1 = 1500;
                TIM3->CCR2 = 1500;
            }
        }
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
  * @brief KeypadTask: 키패드 스캔 후 큐 전송
  */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint16_t pressed_keys = 0;
  static uint16_t prev_keys = 0;
  TurretMsg_t tx_msg;

  for(;;)
  {
    pressed_keys = Scan_Keypad();

    uint16_t transition = pressed_keys & ~prev_keys;
    prev_keys = pressed_keys;

    if(pressed_keys != 0 || transition != 0)
    {
        tx_msg.msg_type   = TYPE_MANUAL;
        tx_msg.manual_cmd = 0;

        if(transition & CMD_TOGGLE) tx_msg.manual_cmd |= CMD_TOGGLE;
        tx_msg.manual_cmd |= (pressed_keys & (CMD_UP | CMD_DOWN | CMD_LEFT | CMD_RIGHT));

        if(tx_msg.manual_cmd != 0)
            osMessageQueuePut(KeyQueueHandle, &tx_msg, 0, 0);
    }

    osDelay(30);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief UART 수신 완료 콜백 (ISR 컨텍스트)
  *
  * ISR에서는 버퍼에 글자를 쌓고, 문장이 완성되면 uart_parse_buf에 복사 후
  * uart_msg_ready 플래그만 1로 세운다. 실제 sscanf 파싱은 MotorTask에서 수행.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if(rx_byte == '\n' || rx_byte == '\r')
        {
            if(rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';
                memcpy(uart_parse_buf, rx_buffer, rx_index + 1);
                uart_msg_ready = 1;
                rx_index = 0;
            }
        }
        else
        {
            if(rx_index < sizeof(rx_buffer) - 1)
                rx_buffer[rx_index++] = rx_byte;
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
/* USER CODE END Application */
