/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
extern osMessageQueueId_t KeyQueueHandle;
extern TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TYPE_MANUAL  0
#define TYPE_AUTO    1

#define CMD_UP     (1 << 0)
#define CMD_DOWN   (1 << 1)
#define CMD_LEFT   (1 << 2)
#define CMD_RIGHT  (1 << 3)
#define CMD_TOGGLE (1 << 4)

// freertos.c와 동일한 구조체를 main.c에서도 알 수 있도록 선언
typedef struct {
    uint8_t  msg_type;
    uint16_t manual_cmd;
    int16_t  error_x;
    int16_t  error_y;
} TurretMsg_t;

uint16_t Scan_Keypad(void)
{
    uint16_t result = 0;

    const uint8_t keys[4][4] = {
        { 1,  2,  3,  4},
        { 5,  6,  7,  8},
        { 9, 10, 11, 12},
        {13, 14, 15, 16}
    };

    GPIO_TypeDef* row_ports[4] = {GPIOA, GPIOB, GPIOB, GPIOB};
    uint16_t      row_pins[4]  = {GPIO_PIN_10, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_4};

    GPIO_TypeDef* col_ports[4] = {GPIOB, GPIOA, GPIOA, GPIOC};
    uint16_t      col_pins[4]  = {GPIO_PIN_10, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_7};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  GPIO_PIN_SET);

    for(int row = 0; row < 4; row++)
    {
        HAL_GPIO_WritePin(row_ports[row], row_pins[row], GPIO_PIN_RESET);
        osDelay(2);

        for(int col = 0; col < 4; col++)
        {
            if(HAL_GPIO_ReadPin(col_ports[col], col_pins[col]) == GPIO_PIN_RESET)
            {
                if(keys[row][col] == 1)  result |= CMD_TOGGLE;
                if(keys[row][col] == 2)  result |= CMD_UP;
                if(keys[row][col] == 10) result |= CMD_DOWN;
                if(keys[row][col] == 5)  result |= CMD_LEFT;
                if(keys[row][col] == 7)  result |= CMD_RIGHT;
            }
        }
        HAL_GPIO_WritePin(row_ports[row], row_pins[row], GPIO_PIN_SET);
    }

    return result;
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
  MX_TIM3_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canFilterConfig;

  // 모든 CAN ID 메시지를 수신하도록 마스크 필터 개방
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
      Error_Handler();
  }

  // CAN 컨트롤러 시작
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }

  // RX0 인터럽트(수신 알림) 활성화
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/* USER CODE BEGIN 4 */
/**
  * @brief CAN 수신 인터럽트 콜백 함수
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    TurretMsg_t rx_msg;

    // FIFO0에서 메시지 읽기
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        /* * [수신 데이터 파싱]
         * CAN 버스로부터 들어온 8바이트 배열(RxData)을 TurretMsg_t 구조체에 맞게 조립합니다.
         */
        rx_msg.msg_type   = RxData[0];
        rx_msg.manual_cmd = (uint16_t)((RxData[2] << 8) | RxData[1]);
        rx_msg.error_x    = (int16_t)((RxData[4] << 8) | RxData[3]);
        rx_msg.error_y    = (int16_t)((RxData[6] << 8) | RxData[5]);

        /*
         * [FreeRTOS 큐 전송]
         * 파싱 완료된 구조체를 KeyQueueHandle에 넣어 MotorTask가 처리하도록 합니다.
         */
        if (KeyQueueHandle != NULL) {
            osMessageQueuePut(KeyQueueHandle, &rx_msg, 0, 0);
        }
    }
}
/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
