# Target Tracking System

카메라 기반 객체 추적과 STM32 제어를 결합한 실시간 자동 추적 시스템입니다.
이 프로젝트는 컴퓨터 비전으로 목표를 탐지하고, 추적 결과를 STM32 보드로 전달해 모터 제어까지 연결하는 흐름을 구현했습니다.

## Project Overview

이 시스템은 다음 과정을 통해 동작합니다.

- 카메라 입력에서 목표 객체를 검출합니다.
- OpenCV를 활용해 객체의 중심 좌표를 추출합니다.
- 칼만 필터를 적용해 위치 추적의 안정성을 높입니다.
- 추적 결과를 STM32로 전송해 모터 제어를 수행합니다.
- 수동 모드와 자동 추적 모드를 전환할 수 있습니다.

## Key Features

- 실시간 객체 탐지 및 추적
- HSV 기반 색상 인식
- 칼만 필터를 이용한 위치 보정
- CAN/UART 통신을 통한 제어 신호 전달
- FreeRTOS 기반 멀티태스크 제어
- 수동/자동 모드 전환 기능

## System Architecture

```mermaid
flowchart LR
    A[📷 Camera Input] --> B[👁️ Object Detection]
    B --> C[🧠 Kalman Filter]
    C --> D[📡 Send Target Data]
    D --> E[🔧 STM32 Signal Receive]
    E --> F[⚙️ FreeRTOS Control Task]
    F --> G[🎯 Motor PWM Control]
    F --> H[⌨️ Keypad Input]

    style A fill:#e8f1ff,stroke:#4a90e2,stroke-width:2px
    style B fill:#f3e8ff,stroke:#8e44ad,stroke-width:2px
    style C fill:#e8f8f1,stroke:#27ae60,stroke-width:2px
    style D fill:#fff4e5,stroke:#f39c12,stroke-width:2px
    style E fill:#fce8e6,stroke:#e74c3c,stroke-width:2px
    style F fill:#f0f0f0,stroke:#2c3e50,stroke-width:2px
    style G fill:#eaf7ea,stroke:#2ecc71,stroke-width:2px
    style H fill:#fef9e7,stroke:#f1c40f,stroke-width:2px
```

### Hardware / Software Roles

- Raspberry Pi / Linux Environment
  - 카메라 입력 처리
  - 목표 탐지 및 좌표 계산
  - CAN 또는 UART를 통해 STM32로 제어 명령 전송

- STM32F4 / CubeMX Project
  - CAN 수신 및 메시지 파싱
  - FreeRTOS 태스크 기반 모터 제어
  - 키패드 입력을 통한 수동/자동 모드 전환

## Project Structure

```text
target_tracking_system/
├── README.md
├── Member_B.md
├── motor_project/
│   ├── motor_project.ioc
│   ├── Core/
│   │   ├── Inc/
│   │   └── Src/
│   │       ├── main.c
│   │       ├── freertos.c
│   │       ├── can.c
│   │       └── stm32f4xx_it.c
│   └── Drivers/
│       └── STM32F4xx_HAL_Driver/
├── ras/
│   ├── CMakeLists.txt
│   ├── face_detect.cpp
│   ├── face_detect2.cpp
│   └── face_detect
```

## Core Files

- [ras/CMakeLists.txt](ras/CMakeLists.txt)
  - Raspberry Pi용 OpenCV 빌드 환경을 구성합니다.

- [ras/face_detect.cpp](ras/face_detect.cpp)
  - 목표 객체를 탐지하고 칼만 필터로 추적한 뒤 CAN으로 STM32에 전송합니다.

- [ras/face_detect2.cpp](ras/face_detect2.cpp)
  - UART 기반 전송 방식을 실험하기 위한 대체 구현입니다.

- [motor_project/Core/Src/main.c](motor_project/Core/Src/main.c)
  - STM32의 초기화, CAN 설정, 수신 처리의 진입점입니다.

- [motor_project/Core/Src/freertos.c](motor_project/Core/Src/freertos.c)
  - FreeRTOS 기반 모터 제어 태스크와 키패드 입력 처리 로직을 포함합니다.

- [motor_project/motor_project.ioc](motor_project/motor_project.ioc)
  - CubeMX 설정 파일로, 펌웨어 구성을 재생성할 때 사용됩니다.

## Workflow

1. 카메라로 프레임을 읽습니다.
2. 노란색 객체를 HSV 색공간에서 검출합니다.
3. 윤곽선 기반으로 중심 좌표를 계산합니다.
4. 칼만 필터로 추적 결과를 안정화합니다.
5. 좌표 오차를 STM32로 전송합니다.
6. STM32는 수신값을 바탕으로 서보/모터 PWM 값을 갱신합니다.
7. 키패드를 통해 수동 조작 또는 자동 추적 모드를 전환할 수 있습니다.

## Build & Run

### Raspberry Pi / Linux

Required Packages

- OpenCV
- CMake
- C++17 compatible compiler

Example Build

```bash
cd ras
cmake -S . -B build
cmake --build build
```

실행

```bash
./build/face_detect
```


### STM32 펌웨어

1. STM32CubeIDE 또는 STM32CubeMX를 실행합니다.
2. [motor_project/motor_project.ioc](motor_project/motor_project.ioc) 를 열어 프로젝트를 다시 생성합니다.
3. 빌드 후 보드에 플래시합니다.

## Technical Highlights

- HSV 기반 색상 검출
- 칼만 필터를 통한 목표 위치 안정화
- CAN/UART 기반 통신 구현
- FreeRTOS를 활용한 멀티태스크 제어
- 수동 모드와 자동 추적 모드 전환

## Project Results

이번 프로젝트를 통해 다음과 같은 결과를 도출했습니다.

- 카메라 입력 기반 목표 객체를 실시간으로 추적하는 흐름을 구현했습니다.
- OpenCV와 STM32 간 통신을 연결해, 인식 결과가 실제 모터 제어로 이어지도록 구성했습니다.
- 칼만 필터를 적용해 추적의 흔들림을 줄이고 안정적인 제어에 기여했습니다.
- FreeRTOS 기반 태스크 구조를 적용해 제어 로직과 입력 처리의 책임을 분리했습니다.
- 수동 제어와 자동 추적 모드를 함께 지원해, 시스템의 확장성과 실험성을 높였습니다.

## Future Improvements

- 실시간 시각화 화면 추가
- 더 다양한 객체/색상 검출
- PID 제어 고도화
- 로봇/포탑 제어와 연동
- 데이터 로깅 및 추적 성능 분석
