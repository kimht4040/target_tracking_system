#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp> // 칼만 필터용 헤더 추가
#include <iostream>
#include <chrono>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <fcntl.h>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <cstdint>
#include <cstring>

// ==========================================
// 1. UART 통신 클래스 (안정화 패치 적용 완료)
// ==========================================
class UartTransmitter
{
private:
    int fd;

public:
    UartTransmitter(const char *port = "/dev/ttyACM0")
    {
        fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            std::cerr << "UART 포트 열기 실패. 권한(sudo)이나 포트 이름(" << port << ")을 확인하세요." << std::endl;
            exit(1);
        }

        struct termios options;
        if (tcgetattr(fd, &options) != 0)
        {
            std::cerr << "UART 설정 읽기 실패: " << std::strerror(errno) << std::endl;
            close(fd);
            exit(1);
        }

        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        cfmakeraw(&options);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            std::cerr << "UART 설정 적용 실패: " << std::strerror(errno) << std::endl;
            close(fd);
            exit(1);
        }

        // ⭐ UART 안정화 패치 1: 파이프에 낀 쓰레기 데이터 청소
        tcflush(fd, TCIOFLUSH);

        std::cout << port << " UART 연결 성공! 통신을 시작합니다." << std::endl;
    }

    ~UartTransmitter()
    {
        if (fd != -1)
            close(fd);
    }

    bool sendErrorData(int16_t error_x, int16_t error_y)
    {
        uint8_t buffer[6];
        buffer[0] = 0xFF;
        buffer[1] = error_x & 0xFF;
        buffer[2] = (error_x >> 8) & 0xFF;
        buffer[3] = error_y & 0xFF;
        buffer[4] = (error_y >> 8) & 0xFF;
        buffer[5] = 0xFE;

        ssize_t written = write(fd, buffer, sizeof(buffer));
        if (written != static_cast<ssize_t>(sizeof(buffer)))
        {
            std::cerr << "UART 전송 실패: " << std::strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    bool readPacket(int16_t &error_x, int16_t &error_y, int timeout_ms = 5)
    {
        uint8_t packet[6];
        int packetIndex = 0;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (std::chrono::steady_clock::now() < deadline)
        {
            uint8_t byte = 0;
            ssize_t received = read(fd, &byte, 1);

            if (received == 1)
            {
                if (packetIndex == 0 && byte != 0xFF)
                    continue;
                packet[packetIndex++] = byte;

                if (packetIndex == 6)
                {
                    if (packet[0] == 0xFF && packet[5] == 0xFE)
                    {
                        error_x = static_cast<int16_t>(packet[1] | (packet[2] << 8));
                        error_y = static_cast<int16_t>(packet[3] | (packet[4] << 8));
                        return true;
                    }
                    packetIndex = 0;
                }
            }
            else
            {
                usleep(1000);
            }
        }
        return false;
    }
};

// ==========================================
// 2. 칼만 필터 클래스 (추적 예측 알고리즘)
// ==========================================
class TargetTrackerKF
{
private:
    cv::KalmanFilter kf;
    cv::Mat meas;
    bool initialized;

public:
    TargetTrackerKF()
    {
        kf = cv::KalmanFilter(4, 2, 0);
        meas = cv::Mat(2, 1, CV_32F);
        initialized = false;

        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                               0, 1, 0, 1,
                               0, 0, 1, 0,
                               0, 0, 0, 1);

        kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                0, 1, 0, 0);

        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    }

    cv::Point2f update(float measure_x, float measure_y, bool object_detected)
    {
        if (!initialized && object_detected)
        {
            kf.statePost.at<float>(0) = measure_x;
            kf.statePost.at<float>(1) = measure_y;
            kf.statePost.at<float>(2) = 0;
            kf.statePost.at<float>(3) = 0;
            initialized = true;
        }

        cv::Mat prediction = kf.predict();
        cv::Point2f predict_pt(prediction.at<float>(0), prediction.at<float>(1));

        if (object_detected)
        {
            meas.at<float>(0) = measure_x;
            meas.at<float>(1) = measure_y;
            cv::Mat estimated = kf.correct(meas);
            return cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
        }

        return predict_pt; // 타겟을 놓쳤을 때 예측값 반환
    }
};

// ==========================================
// 3. 스레드 공유 데이터 구조체
// ==========================================
struct SharedFrame
{
    cv::Mat frame;
    uint64_t sequence = 0;
    bool hasFrame = false;
};

struct SharedDisplay
{
    cv::Mat frame;
    cv::Mat mask;
    uint64_t sequence = 0;
    bool hasFrame = false;
};

struct TargetError
{
    int16_t x = 0;
    int16_t y = 0;
};

// ==========================================
// 4. 스레드 1: 카메라 캡처 루프
// ==========================================
void captureLoop(std::atomic<bool> &running, SharedFrame &sharedFrame, std::mutex &frameMutex, std::condition_variable &frameCv)
{

    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink drop=true sync=false max-buffers=1";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "에러: 카메라를 열 수 없습니다." << std::endl;
        running = false;
        frameCv.notify_all();
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    while (running)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            sharedFrame.frame = frame.clone();
            sharedFrame.sequence++;
            sharedFrame.hasFrame = true;
        }
        frameCv.notify_one();
    }
    cap.release();
}

// ==========================================
// 5. 스레드 2: 영상 처리 & 칼만 필터 루프
// ==========================================
void processingLoop(std::atomic<bool> &running, SharedFrame &sharedFrame, std::mutex &frameMutex, std::condition_variable &frameCv,
                    SharedDisplay &sharedDisplay, std::mutex &displayMutex, TargetError &pendingError, bool &hasPendingError,
                    std::mutex &uartMutex, std::condition_variable &uartCv)
{
    cv::Mat hsv, mask, mask1, mask2;
    const int centerX = 320;
    const int centerY = 240;
    uint64_t lastSequence = 0;

    TargetTrackerKF tracker; // 칼만 필터 인스턴스 생성

    while (running)
    {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCv.wait(lock, [&]
                         { return !running || (sharedFrame.hasFrame && sharedFrame.sequence != lastSequence); });
            if (!running)
                break;
            frame = sharedFrame.frame.clone();
            lastSequence = sharedFrame.sequence;
        }

        cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 빨간색 검출 (두 구간)
        cv::inRange(hsv, cv::Scalar(0, 70, 40), cv::Scalar(15, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(165, 70, 40), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        int largestContourIndex = -1;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > 500 && area > maxArea)
            {
                maxArea = area;
                largestContourIndex = static_cast<int>(i);
            }
        }

        // 십자선 그리기
        cv::line(frame, cv::Point(centerX - 12, centerY), cv::Point(centerX + 12, centerY), cv::Scalar(255, 255, 255), 1);
        cv::line(frame, cv::Point(centerX, centerY - 12), cv::Point(centerX, centerY + 12), cv::Scalar(255, 255, 255), 1);

        bool targetFound = (largestContourIndex != -1);
        float raw_cx = centerX, raw_cy = centerY;

        if (targetFound)
        {
            cv::Moments M = cv::moments(contours[largestContourIndex]);
            if (M.m00 > 0)
            {
                raw_cx = float(M.m10 / M.m00);
                raw_cy = float(M.m01 / M.m00);

                cv::Rect boundingBox = cv::boundingRect(contours[largestContourIndex]);
                cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 2);
            }
        }

        // ⭐ 칼만 필터 적용 (타겟을 놓쳤어도 예측 좌표가 나옵니다)
        cv::Point2f filtered_pos = tracker.update(raw_cx, raw_cy, targetFound);

        // 필터링된 좌표 기준으로 오차 계산
        int16_t errorX = static_cast<int16_t>(filtered_pos.x - centerX);
        int16_t errorY = static_cast<int16_t>(centerY - filtered_pos.y);

        // 예측 위치 파란 점으로 그리기
        cv::circle(frame, filtered_pos, 5, cv::Scalar(255, 0, 0), -1);

        // UART 스레드에 데이터 넘기기
        {
            std::lock_guard<std::mutex> lock(uartMutex);
            pendingError = {errorX, errorY};
            hasPendingError = true;
        }
        uartCv.notify_one();

        std::string posText = "Target: (" + std::to_string((int)filtered_pos.x) + ", " + std::to_string((int)filtered_pos.y) +
                              ") Err: (" + std::to_string(errorX) + ", " + std::to_string(errorY) + ")";
        cv::putText(frame, posText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        {
            std::lock_guard<std::mutex> lock(displayMutex);
            sharedDisplay.frame = frame.clone();
            sharedDisplay.mask = mask.clone();
            sharedDisplay.sequence++;
            sharedDisplay.hasFrame = true;
        }
    }
}

// ==========================================
// 6. 스레드 3: UART 송수신 루프
// ==========================================
void uartLoop(std::atomic<bool> &running, TargetError &pendingError, bool &hasPendingError, std::mutex &uartMutex, std::condition_variable &uartCv)
{
    UartTransmitter uart_tx("/dev/ttyACM0");

    while (running)
    {
        TargetError error;
        {
            std::unique_lock<std::mutex> lock(uartMutex);
            uartCv.wait(lock, [&]
                        { return !running || hasPendingError; });
            if (!running)
                break;
            error = pendingError;
            hasPendingError = false;
        }

        if (uart_tx.sendErrorData(error.x, error.y))
        {

            // ⭐ UART 안정화 패치 2: STM32가 응답할 시간을 줌 (수신 씹힘 방지)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            int16_t rxX = 0, rxY = 0;
            if (uart_tx.readPacket(rxX, rxY))
            {
                bool matched = (rxX == error.x && rxY == error.y);
                std::cout << "TX x=" << error.x << " y=" << error.y << " | RX x=" << rxX << " y=" << rxY << (matched ? " OK" : " MISMATCH") << std::endl;
            }
            else
            {
                std::cout << "TX x=" << error.x << " y=" << error.y << " | RX timeout" << std::endl;
            }
        }
    }
}

// ==========================================
// 7. 메인 함수
// ==========================================
int main()
{
    std::cout << "칼만 필터 탑재 타겟 트래킹 (30FPS). 종료: 'ESC'" << std::endl;

    std::atomic<bool> running(true);
    SharedFrame sharedFrame;
    SharedDisplay sharedDisplay;
    TargetError pendingError;
    bool hasPendingError = false;

    std::mutex frameMutex, displayMutex, uartMutex;
    std::condition_variable frameCv, uartCv;

    std::thread captureThread(captureLoop, std::ref(running), std::ref(sharedFrame), std::ref(frameMutex), std::ref(frameCv));
    std::thread processingThread(processingLoop, std::ref(running), std::ref(sharedFrame), std::ref(frameMutex), std::ref(frameCv),
                                 std::ref(sharedDisplay), std::ref(displayMutex), std::ref(pendingError), std::ref(hasPendingError),
                                 std::ref(uartMutex), std::ref(uartCv));
    std::thread uartThread(uartLoop, std::ref(running), std::ref(pendingError), std::ref(hasPendingError), std::ref(uartMutex), std::ref(uartCv));

    const int targetDelayMs = 1000 / 30;
    uint64_t lastDisplaySequence = 0;

    while (running)
    {
        cv::Mat displayFrame, displayMask;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            if (sharedDisplay.hasFrame && sharedDisplay.sequence != lastDisplaySequence)
            {
                displayFrame = sharedDisplay.frame.clone();
                displayMask = sharedDisplay.mask.clone();
                lastDisplaySequence = sharedDisplay.sequence;
            }
        }

        if (!displayFrame.empty())
            cv::imshow("Turret Vision - KF Tracking", displayFrame);
        if (!displayMask.empty())
            cv::imshow("Red Mask", displayMask);

        if (cv::waitKey(targetDelayMs) == 27)
        { // ESC 누르면 종료
            running = false;
            frameCv.notify_all();
            uartCv.notify_all();
            break;
        }
    }

    running = false;
    frameCv.notify_all();
    uartCv.notify_all();

    if (captureThread.joinable())
        captureThread.join();
    if (processingThread.joinable())
        processingThread.join();
    if (uartThread.joinable())
        uartThread.join();

    cv::destroyAllWindows();
    return 0;
}