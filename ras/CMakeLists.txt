cmake_minimum_required(VERSION 3.16)
project(face_detect LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 최적화 플래그
# 64비트(aarch64): -mfpu / -mfloat-abi 미지원 → 제거
# 32비트(armv7l) 환경이라면 아래 주석을 해제하고 위 줄을 주석 처리하세요.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -mcpu=cortex-a72")
else()
    # 32비트 armv7l
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -mcpu=cortex-a72 -mfpu=neon-fp-armv8 -mfloat-abi=hard")
endif()

# OpenCV 탐색
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version : ${OpenCV_VERSION}")
message(STATUS "OpenCV include : ${OpenCV_INCLUDE_DIRS}")

add_executable(face_detect face_detect.cpp)
target_include_directories(face_detect PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(face_detect PRIVATE ${OpenCV_LIBS})
