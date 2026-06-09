# Auto-Tracking Pan-Tilt Turret System 

## Overview
웹캠을 통해 실시간으로 특정 객체(사람의 얼굴 또는 특정 색상의 사물)를 검출하고, 2축(Pan-Tilt) 서보모터를 구동하여 객체를 화면 중앙으로 정조준하는 자동 추적 터렛 시스템입니다. 

비전 처리(Vision Processing)와 실시간 하드웨어 제어(Real-time Control) 노드를 분리하여 지연 시간(Latency)을 최소화하였으며, 수동 제어와 자동 추적 모드를 유연하게 전환할 수 있도록 설계되었습니다.

<br/>

## Tech Stack
### Hardware
* **MCU:** STM32 시리즈 (FreeRTOS 기반 실시간 제어)
* **Vision Node:** Ubuntu 데스크톱 환경(얼굴 객체인식을 위해), 라즈베리파이4(공객체 인식)
* **Actuator:** 2-Axis Pan-Tilt Servo Motors
* **Communication:** CAN Transceiver (or USB to TTL)

### Software & Languages
* **Languages:** C, C++
* **Vision Processing:** OpenCV (Haar Cascade, HSV Color Tracking)
* **RTOS:** FreeRTOS (Task & Queue management)
* **Protocol:** CAN (SocketCAN) / UART

<br/>

## Key Features

### 1. 지능형 객체 추적 (Intelligent Object Tracking)
* **AI & Color-based Detection:** OpenCV를 활용하여 노란색 공(HSV 필터링) 및 사람의 얼굴(Haar Cascade)을 실시간(60FPS)으로 검출합니다.
* **Kalman Filter 적용:** 영상 노이즈나 일시적인 객체 가림 현상(Occlusion)이 발생하더라도 궤적을 예측하여 부드럽고 끊김 없는 추적을 보장합니다.

### 2. 정밀한 실시간 모터 제어 (Precise Motor Control)
* **PD 제어 알고리즘 (Proportional-Derivative Control):** 타겟 좌표와 현재 시야 중심 간의 오차(Error)를 계산하여 비례 제어로 이동 속도를 결정합니다. 목표 지점 도달 시 발생하는 오버슈트(Overshoot)와 잔떨림(Hunting)을 최소화하여 군용 터렛 수준의 묵직하고 정확한 제어를 구현했습니다.
* **Deadzone 설정:** 중앙 좌표 부근의 미세한 픽셀 오차에 모터가 반응하지 않도록 데드존을 설정하여 기구적 안정성을 확보했습니다.

### 3. 분산 아키텍처 기반 초고속 통신 (Offloading & Communication)
* **연산 부하 분산:** 연산량이 많은 비전 처리는 Ubuntu PC에서 수행하고, 계산된 오차 좌표(dx, dy)만 STM32로 전송하는 오프로딩(Offloading) 구조를 채택했습니다.
* **CAN / UART 통신:** 16비트 오차 데이터를 8바이트 패킷으로 패킹(Packing)하여 노이즈에 강한 CAN 통신(또는 UART)으로 지연 없이 MCU에 전달합니다.

### 4. 수동/자동 상태 전환 및 메모리 기능 (Mode State Machine)
* **Seamless Mode Switching:** 키패드 인터럽트(Interrupt)를 통해 즉각적으로 수동(Manual) 및 자동(Auto) 모드를 토글할 수 있습니다.
* **Position Memory:** 모드 전환 시 현재의 Pan-Tilt 각도 위치를 기억(Save)하여, 수동 조작 후 다시 자동 모드로 복귀할 때 튀는 현상 없이 자연스럽게 동작을 이어나갑니다.

### 5. 타겟 적중 피드백 (Target Hit Feedback)
* **GPIO 제어:** 추적 중인 객체가 시야의 중앙(Deadzone 이내)에 완벽하게 록온(Lock-on)되면, STM32의 GPIO 핀을 제어하여 즉각적으로 LED를 점등시켜 사용자에게 시각적 타격 피드백을 제공합니다.

<br/>

## System Architecture (Brief)
1. **[Vision Node - Ubuntu/C++]** 웹캠 영상 획득 → 객체 검출(OpenCV) → 중앙 픽셀과의 오차(dx, dy) 계산 → 데이터 패킹 및 CAN/UART 송신
2. **[Control Node - STM32/FreeRTOS]** 통신 수신 인터럽트 → Message Queue로 데이터 전달 → PD 제어 알고리즘을 통한 PWM Duty Cycle 연산 → 2축 서보모터 구동 및 LED 제어