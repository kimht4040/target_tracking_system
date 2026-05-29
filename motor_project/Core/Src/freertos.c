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

// [FIX] ISR → Task 데이터 전달용 변수
// ISR에서는 플래그만 세우고, sscanf는 MotorTask에서 처리
volatile uint8_t uart_msg_ready = 0;   // 완성된 문장이 있으면 1
char             uart_parse_buf[30];   // ISR이 복사해 둔 완성 문자열
/* USER CODE END Variables */

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,          // [FIX] sscanf 스택 사용 대비 256으로 증가
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

  const uint16_t step_size  = 16;

  // ── [변경] 제어 상수 및 안전장치 값 세팅 ──────────────────
  const float    Kp_x       = 0.08f;
  const float    Kp_y       = 0.08f;

  // PD제어 D항(오차 변화율, 가까워질수록 브레이크 기능)
  const float	Kd_x = 0.25f;
  const float	Kd_y = 0.25f;

  const float  K_sq_x     = 0.003f;  //비선형 제곱 게인 (멀수록 폭발적 가속)
  const float  K_sq_y     = 0.003f;

  const int      max_speed  = 40;
  const int      deadzone   = 10; // 서보 버징 방지
  // ─────────────────────────────────────────────────────────

  const float max_pulse = 2300.0f;
  const float min_pulse = 700.0f;

  // D항 계산용
  float prev_error_x = 0.0f;
  float prev_error_y = 0.0f;
  float pulse_x_f = 1500.0f;
  float pulse_y_f = 1500.0f;

  uint8_t current_mode = TYPE_MANUAL;   // 초기값: 수동 모드로 시작

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  TIM3->CCR1 = pulse_x;
  TIM3->CCR2 = pulse_y;

  // UART 수신 인터럽트 시작
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  for(;;)
  {
    // -------------------------------------------------------
    // [FIX] UART 파싱을 태스크(여기)에서 처리
    //       ISR에서 sscanf를 쓰면 HardFault 발생 → 플래그 방식으로 변경
    // -------------------------------------------------------
    if(uart_msg_ready)
    {
        uart_msg_ready = 0;                   // 플래그 즉시 클리어

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

    // -------------------------------------------------------
    // [FIX] osWaitForever → 타임아웃 10ms
    //       osWaitForever는 UART 플래그를 폴링할 기회를 없앰
    // -------------------------------------------------------
    if(osMessageQueueGet(KeyQueueHandle, &rx_msg, NULL, 10) == osOK)
    {
        // 1. 모드 전환 (1번 버튼)
        if(rx_msg.msg_type == TYPE_MANUAL && (rx_msg.manual_cmd & CMD_TOGGLE))
        {
            current_mode = (current_mode == TYPE_AUTO) ? TYPE_MANUAL : TYPE_AUTO;
            // 모드 전환 시 LED off
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
        }

        // 2. 수동 모드 동작
        if(current_mode == TYPE_MANUAL)
        {
            if(rx_msg.msg_type == TYPE_MANUAL)
            {
                if(rx_msg.manual_cmd & CMD_LEFT)  pulse_x -= step_size;
                if(rx_msg.manual_cmd & CMD_RIGHT) pulse_x += step_size;
                if(rx_msg.manual_cmd & CMD_UP)    pulse_y -= step_size;
                if(rx_msg.manual_cmd & CMD_DOWN)  pulse_y += step_size;
            }
        }
        // 3. 자동 모드 동작 (속도 제한 및 데드존 반영)
                else if(current_mode == TYPE_AUTO)
                {
                    if(rx_msg.msg_type == TYPE_AUTO)
                    {
                        // 데드존 처리 (설정한 deadzone 픽셀 이내 오차는 무시)
                        if(rx_msg.error_x > -deadzone && rx_msg.error_x < deadzone) rx_msg.error_x = 0;
                        if(rx_msg.error_y > -deadzone && rx_msg.error_y < deadzone) rx_msg.error_y = 0;

                        // 데드존 진입 판전 LED 전원키기 > 나중에 레이저포인터로 변경예정
                        if(rx_msg.error_x==0 && rx_msg.error_y ==0)
                        	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
                        else
                        	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);

                        float ex = (float)rx_msg.error_x;
                        float ey = (float)rx_msg.error_y;

                        // ── 비선형 PD 제어 ──────────────────────────────
                        // P항:  선형 (방향 유지)
                        // P²항: 오차 제곱 (멀수록 폭발적, 가까울수록 급감)
                        // D항:  미분 브레이크 (오버슈트 억제)

                        float sign_x = (ex >= 0) ? 1.0f : -1.0f;
                        float sign_y = (ey >= 0) ? 1.0f : -1.0f;

                        float delta_x = Kp_x  * ex                      // 선형항
                                      + K_sq_x * sign_x * (ex * ex)     // 비선형항
                                      + Kd_x  * (ex - prev_error_x);    // 브레이크항

                        float delta_y = Kp_y  * ey
                                      + K_sq_y * sign_y * (ey * ey)
                                      + Kd_y  * (ey - prev_error_y);


                        prev_error_x = ex;
                        prev_error_y = ey;

                        // 🚨 한 번에 과도하게 회전하는 것을 막는 스피드 리미터
                        if(delta_x > (float)max_speed)  delta_x = (float)max_speed;
                        if(delta_x < -(float)max_speed) delta_x = -(float)max_speed;
                        if(delta_y > (float)max_speed)  delta_y = (float)max_speed;
                        if(delta_y < -(float)max_speed) delta_y = -(float)max_speed;

                        // float 누적 후 레지스터 반영
                        pulse_x_f += delta_x;
                        pulse_y_f -= delta_y;

                        pulse_x = (uint16_t)pulse_x_f;
                        pulse_y = (uint16_t)pulse_y_f;
                    }
                    else
                    {
                        // 자동 모드 상태인데 유효한 오차 값이 안 들어오면 중앙 대기
                        pulse_x_f = 1500.0f;
                        pulse_y_f = 1500.0f;

                        pulse_x = 1500;
                        pulse_y = 1500;

                        prev_error_x = 0.0f;
                        prev_error_y = 0.0f;

                        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
                    }
                }

        // 소프트 리미트
        if(pulse_x > max_pulse) pulse_x = max_pulse;
        if(pulse_x < min_pulse) pulse_x = min_pulse;
        if(pulse_y > max_pulse) pulse_y = max_pulse;
        if(pulse_y < min_pulse) pulse_y = min_pulse;

        TIM3->CCR1 = (uint16_t)pulse_x_f;
        TIM3->CCR2 = (uint16_t)pulse_y_f;
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
  * [FIX] sscanf는 내부적으로 힙(malloc)을 사용하므로 ISR에서 호출 시 HardFault 발생.
  *       ISR에서는 버퍼에 글자를 쌓고, 문장이 완성되면 uart_parse_buf에 복사 후
  *       uart_msg_ready 플래그만 1로 세운다. 실제 sscanf 파싱은 MotorTask에서 수행.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // 생존 신고: 수신마다 LED 토글
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if(rx_byte == '\n' || rx_byte == '\r')
        {
            if(rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';

                // [FIX] sscanf 제거 → 버퍼만 복사 후 플래그 세팅
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

        // 다음 바이트 수신 대기 재등록
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
/* USER CODE END Application */
