# 객체 추적 시스템

## Member B

- **FreeRTOS 환경 구축:** STM32에 RTOS를 올리고 UART 수신, CAN 송신, 모터 제어 태스크(Task) 분리 및 우선순위 할당.
        
- **통신 인터페이스 구현 (HAL):** 호스트로부터 UART로 목표 좌표/제어 명령 수신, 서보 모터(또는 모터 드라이버)로 CAN 통신 신호 송신.
        
- **하드웨어 제어:** 수신된 좌표 오차를 기반으로 2축 서보 모터 구동 로직 작성, 조준 완료 시 레이저 포인터 격발(GPIO 제어).

### 개발 환경 구축

- STM32CUBE IDE 설치
    - 기반 기술: 전 세계적으로 유명한 'Eclipse' 에디터와 'GCC' 컴파일러를 기반으로 합니다.

    - MX 내장: 예전에는 두 프로그램을 따로 썼지만, 이제는 IDE 안에 MX가 들어와 있어서 탭 하나로 왔다 갔다 하며 설정을 바꿀 수 있습니다.

    - 강력한 디버깅: 실시간으로 MCU 내부 변수 값을 확인하거나, 코드를 한 줄씩 실행하며 논리 오류를 잡는 데 최적화되어 있습니다.

    - 빌드 시스템: 쓴 코드를 기계어(Binary/Hex)로 바꿔서 실제 칩에 구워주는 역할을 합니다.

- STM32CUBE MX 설치
    - 핀 마이크로 매니징: STM32는 핀 하나가 여러 기능을 수행할 수 있습니다. (예: PA9 핀을 일반 입출력으로 쓸지, 통신용 UART로 쓸지 마우스 클릭으로 결정)

    - 클럭(Clock) 설정: 컴퓨터의 CPU 속도를 조절하듯, MCU 내부의 심장 박동수를 시각적인 다이어그램을 보며 설정합니다.

    - 코드 생성(Code Generation): 설정을 마치고 버튼을 누르면, 복잡한 레지스터 설정을 자동으로 해주는 C언어 초기화 코드를 생성해 줍니다.

    - 미들웨어 통합: FreeRTOS, TCP/IP, USB 스택 같은 복잡한 소프트웨어를 클릭 몇 번으로 프로젝트에 포함시킬 수 있습니다.

- FreeRTOS(Real-Time Operating System)
    - Task(태스크): 프로그램을 기능 단위로 쪼갠 것 (예: 통신 태스크, 모터 제어 태스크).

    - 스케줄링: 우선순위가 높은 일을 먼저 처리합니다. 타겟 추적 프로젝트에서는 '모터 제어'가 'UI 업데이트'보다 우선순위가 높아야 끊김 없는 추적이 가능합니다.

    - Queue/Semaphore: 태스크끼리 데이터를 안전하게 주고받거나 자원을 선점하기 위한 도구입니다.

- HAL(Hardware Abstraction Layer)
    - 복잡한 MCU의 레지스터를 몰라서 함수 호출 한 번으로 제어할 수 있게 도와주는 라이브러리


#### STM32CubeMX 초기 설정

* **SYS (System):**
    * `Debug`: **Serial Wire** (디버깅 및 펌웨어 다운로드 활성화)
    * `Timebase Source`: **TIM1** 변경 (중요: FreeRTOS의 SysTick 점유로 인한 충돌 방지)
* **RCC (Clock):**
    * `High Speed Clock (HSE)`: **Crystal/Ceramic Resonator** (외부 클럭 소스 사용)

* ⚠️ 트러블슈팅: Task 함수 위치 확인
- **현상**: `freertos.c`가 아닌 `main.c` 파일 하단에 Task Entry 함수가 생성됨.
- **원인**: CubeMX 설정 중 'Keep User Code when re-generating' 옵션이나 'Generate peripheral initialization as a pair of .c/.h files' 옵션 설정 차이로 인한 현상.
- **조치**: `main.c` 하단의 `StartDefaultTask` 내부에 LED 제어 로직을 작성하여 시스템 정상 동작 확인.
- **학습**: 프로젝트 규모가 커지면 파일 분할 옵션을 활성화하여 `main.c`의 비대화를 방지하는 것이 효율적임.

##### Connectivity (통신) 설정
* **USART2 (Member A 협업용):**
    * `Mode`: **Asynchronous** (비동기 방식)
    * `Baud Rate`: **115200 bps**
    * **NVIC Settings**: `USART2 global interrupt` **Enabled** 체크 (데이터 수신 실시간성 확보를 위한 인터럽트 필수 활성화)

#####  Middleware 설정
* **FreeRTOS:**
    * `Interface`: **CMSIS_V2**
    * `Task 설정`: 기본 `defaultTask` 생성 확인 (향후 시스템 확장에 따라 `UartTask`, `MotorTask` 추가 예정)

#####  Clock Configuration
* **HCLK (MHz)**: **180**
    * F446RE의 최대 동작 클럭으로 설정하여 연산 및 처리 속도 최적화


## 2일차

- 라즈베리파이5 STM32 F446RE MG90S 2개

### STM32CubeMX 추가 설정
- TIMER 설정 (PWM 출력용)
    - 왼쪽 메뉴 Timers ➡️ TIM2 클릭
    
    - Clock Source: Internal Clock 선택

    - Channel 1: PWM Generation CH1 선택 (선택 시 PA0 핀이 파란색으로 활성화됩니다.)

    - Parameter Settings (하단 탭):

    - Prescaler (PSC): 1799

    - Counter Period (ARR): 999

코드 생성:

우측 상단 GENERATE CODE 버튼을 눌러 CubeIDE 프로젝트로 설정을 반영합니다.

### 수동 서보 모터 돌리기
- 디버깅 실패
