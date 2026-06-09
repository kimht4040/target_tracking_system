/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

// [삭제 완료] UART 관련 수신 버퍼 및 플래그 변수 모두 제거되었습니다.
/* USER CODE END Variables */

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
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

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {
  /* Create the queue(s) */
  KeyQueueHandle = osMessageQueueNew (8, sizeof(TurretMsg_t), &KeyQueue_attributes);

  /* Create the thread(s) */
  MotorTaskHandle = osThreadNew(StartDefaultTask, NULL, &MotorTask_attributes);
  KeypadTaskHandle = osThreadNew(StartTask02, NULL, &KeypadTask_attributes);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  MotorTask: 서보 제어 및 메시지 큐 처리
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TurretMsg_t rx_msg;

  uint16_t pulse_x = 1500;
  uint16_t pulse_y = 1500;

  const uint16_t step_size = 5;

  // ── 제어 상수 ────────────────────────────────────────────
  const float Kp_x      = 0.08f;
  const float Kp_y      = 0.08f;
  const float Kd_x      = 0.25f;
  const float Kd_y      = 0.25f;
  const int   max_speed = 15;
  const int   deadzone  = 20;
  const float max_pulse = 2300.0f;
  const float min_pulse =  700.0f;
  // ─────────────────────────────────────────────────────────

  float prev_error_x = 0.0f;
  float prev_error_y = 0.0f;
  float pulse_x_f    = 1500.0f;
  float pulse_y_f    = 1500.0f;

  uint8_t current_mode = TYPE_MANUAL;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  TIM3->CCR1 = pulse_x;
  TIM3->CCR2 = pulse_y;

  // [삭제 완료] HAL_UART_Receive_IT 호출문이 제거되었습니다.

  for(;;)
  {
    // [삭제 완료] 기존에 존재하던 uart_msg_ready 문자열 파싱 블록이 제거되었습니다.
    // 앞으로는 CAN 수신 콜백 함수에서 hcan을 통해 받은 데이터를
    // 바로 KeyQueueHandle에 넣어 제어하게 됩니다.

    if(osMessageQueueGet(KeyQueueHandle, &rx_msg, NULL, 20) == osOK)
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

                if(ex > -(float)deadzone && ex < (float)deadzone) ex = 0.0f;
                if(ey > -(float)deadzone && ey < (float)deadzone) ey = 0.0f;

                if(ex == 0.0f && ey == 0.0f)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                else
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

                // ── PD 제어 ──
                float delta_x = Kp_x * ex + Kd_x * (ex - prev_error_x);
                float delta_y = Kp_y * ey + Kd_y * (ey - prev_error_y);

                prev_error_x = ex;
                prev_error_y = ey;

                if(delta_x >  (float)max_speed) delta_x =  (float)max_speed;
                if(delta_x < -(float)max_speed) delta_x = -(float)max_speed;
                if(delta_y >  (float)max_speed) delta_y =  (float)max_speed;
                if(delta_y < -(float)max_speed) delta_y = -(float)max_speed;

                pulse_x_f += delta_x;
                pulse_y_f -= delta_y;

                if(pulse_x_f > max_pulse) pulse_x_f = max_pulse;
                if(pulse_x_f < min_pulse) pulse_x_f = min_pulse;
                if(pulse_y_f > max_pulse) pulse_y_f = max_pulse;
                if(pulse_y_f < min_pulse) pulse_y_f = min_pulse;

                TIM3->CCR1 = (uint16_t)pulse_x_f;
                TIM3->CCR2 = (uint16_t)pulse_y_f;
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            }
        }
    }
    else
    {
        if(current_mode == TYPE_AUTO)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
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
// [삭제 완료] HAL_UART_RxCpltCallback 인터럽트 콜백 함수가 완전히 제거되었습니다.
/* USER CODE END Application */
