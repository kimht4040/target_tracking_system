/*
 * face_detect.cpp — 노란 공 인식 + 칼만 필터
 * 빌드: make  /  실행: make run
 * 종료: Q 또는 ESC
 */

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static constexpr int CAM_W = 640; // USB 웹캠 속도 우선: 필요하면 640으로 복구
static constexpr int CAM_H = 480; // USB 웹캠 속도 우선: 필요하면 480으로 복구
static constexpr int CAM_FPS = 60;
static constexpr int USB_CAMERA_INDEX = -1;      // USB 웹캠 번호: -1은 자동 검색, /dev/video1 고정이면 1
static constexpr float PROCESS_SCALE = 0.5f;     // 노란 마스크 처리 배율: 낮추면 빠름, 너무 낮추면 검출력 감소
static constexpr int DETECT_INTERVAL_FRAMES = 1; // 검출 주기: 1은 매 프레임, 2는 한 프레임 건너뛰며 속도 개선
static constexpr int PRINT_INTERVAL_FRAMES = 10; // 터미널 출력 주기: UART 전송은 매 프레임 유지
static constexpr float UART_OUTPUT_SCALE = 1.0f; // UART 전송값 배율: 테스트 후 0.5f 등으로 조정
static constexpr double FPS_SMOOTHING = 0.90;    // 화면 FPS 표시 안정화 계수
static constexpr bool SHOW_DEBUG_MASK = true;    // true로 바꾸면 노란색 검출 마스크 화면 표시
static constexpr double MIN_RECT_AREA = 80.0;    // 너무 작은 노란 잡음 제거
static constexpr double MIN_RECT_FILL = 0.45;    // 채움 비율: 낮추면 일부 가려진 공도 허용
static const cv::Scalar YELLOW_LO(18, 120, 150); // RGB(235,192,72) ~= HSV(22,177,235)
static const cv::Scalar YELLOW_HI(27, 255, 255);

// ── 칼만 필터 ────────────────────────────────
// 상태 [cx, cy, vx, vy] / 측정 [cx, cy]
class KalmanTracker
{
public:
    KalmanTracker() : kf_(4, 2, 0), initialized_(false)
    {
        kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-2));  // Q 올림 → 움직임 변화에 더 빠르게 반응
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(2)); // R 낮춤 → 검출 중심을 더 타이트하게 추종
        cv::setIdentity(kf_.errorCovPost, cv::Scalar(1.0));
        meas_ = cv::Mat::zeros(2, 1, CV_32F);
    }

    cv::Point2f update(cv::Point2f m)
    {
        meas_.at<float>(0) = m.x;
        meas_.at<float>(1) = m.y;
        if (!initialized_)
        {
            kf_.statePost.at<float>(0) = m.x;
            kf_.statePost.at<float>(1) = m.y;
            kf_.statePost.at<float>(2) = 0;
            kf_.statePost.at<float>(3) = 0;
            initialized_ = true;
        }
        kf_.predict();
        cv::Mat c = kf_.correct(meas_);
        return {c.at<float>(0), c.at<float>(1)};
    }

    cv::Point2f last() const
    {
        if (!initialized_)
            return {-1, -1};
        return {kf_.statePost.at<float>(0), kf_.statePost.at<float>(1)};
    }

    void reset() { initialized_ = false; }

private:
    cv::KalmanFilter kf_;
    cv::Mat meas_;
    bool initialized_;
};

// ── UART 설정 ────────────────────────────────
int uartOpen(const char *dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        perror("[UART] open 실패");
        return -1;
    }

    struct termios opt;
    tcgetattr(fd, &opt);
    cfsetispeed(&opt, baud);
    cfsetospeed(&opt, baud);
    opt.c_cflag = (opt.c_cflag & ~CSIZE) | CS8; // 8비트
    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_cflag &= ~(PARENB | CSTOPB); // 패리티 없음, 스톱비트 1
    opt.c_iflag = IGNPAR;
    opt.c_oflag = 0;
    opt.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &opt);
    return fd;
}

void configureCamera(cv::VideoCapture &cap, int fourcc)
{
    cap.set(cv::CAP_PROP_FOURCC, fourcc);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAM_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_H);
    cap.set(cv::CAP_PROP_FPS, CAM_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
}

bool canReadFrame(cv::VideoCapture &cap)
{
    cv::Mat testFrame;
    for (int i = 0; i < 10; ++i)
    {
        if (cap.read(testFrame) && !testFrame.empty())
            return true;
        usleep(30000);
    }
    return false;
}

bool openUsbCamera(cv::VideoCapture &cap)
{
    std::vector<int> cameraIndexes;
    if (USB_CAMERA_INDEX >= 0)
    {
        cameraIndexes.push_back(USB_CAMERA_INDEX);
    }
    else
    {
        for (int i = 0; i <= 9; ++i)
            cameraIndexes.push_back(i);
    }

    const std::vector<std::pair<const char *, int>> formats = {
        {"MJPG", cv::VideoWriter::fourcc('M', 'J', 'P', 'G')},
        {"YUYV", cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V')},
    };

    for (int index : cameraIndexes)
    {
        for (const auto &format : formats)
        {
            std::cout << "[INFO] USB 웹캠 /dev/video" << index
                      << " " << format.first << " 시도\n";

            cap.release();
            cap.open(index, cv::CAP_V4L2);
            if (!cap.isOpened())
                continue;

            configureCamera(cap, format.second);
            if (canReadFrame(cap))
            {
                std::cout << "[INFO] USB 웹캠 연결 성공: /dev/video"
                          << index << " " << format.first << "\n";
                return true;
            }

            std::cerr << "[WARN] /dev/video" << index
                      << " 열림, 하지만 프레임 읽기 실패\n";
        }
    }

    cap.release();
    return false;
}

// ── 캡처 스레드: USB 웹캠에서 최신 프레임만 유지 ─────────────
class FrameGrabber
{
public:
    explicit FrameGrabber(cv::VideoCapture &cap) : cap_(cap) {}

    void start()
    {
        running_ = true;
        worker_ = std::thread(&FrameGrabber::loop, this);
    }

    void stop()
    {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
    }

    bool getLatest(cv::Mat &out)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasFrame_)
            return false;
        latest_.copyTo(out);
        return true;
    }

    int emptyFrames() const { return emptyFrames_.load(); }

private:
    void loop()
    {
        cv::Mat frame;
        while (running_)
        {
            if (cap_.read(frame) && !frame.empty())
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    frame.copyTo(latest_);
                    hasFrame_ = true;
                }
                emptyFrames_ = 0;
            }
            else
            {
                emptyFrames_++;
                usleep(5000);
            }
        }
    }

    cv::VideoCapture &cap_;
    std::thread worker_;
    std::mutex mutex_;
    cv::Mat latest_;
    std::atomic<bool> running_{false};
    std::atomic<int> emptyFrames_{0};
    bool hasFrame_ = false;
};

// ── 노란 공 검출 ───────────────────────────
struct RectTarget
{
    cv::Point2f center;
    cv::Rect box;
};

bool findYellowBall(const cv::Mat &frame, RectTarget &out)
{
    cv::Mat small, hsv, mask;
    cv::resize(frame, small, cv::Size(), PROCESS_SCALE, PROCESS_SCALE, cv::INTER_AREA);
    cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, YELLOW_LO, YELLOW_HI, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    if (SHOW_DEBUG_MASK)
        cv::imshow("Red mask", mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const float invScale = 1.0f / PROCESS_SCALE;
    double bestScore = 0.0;
    RectTarget best{};

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < MIN_RECT_AREA)
            continue;

        cv::Rect box = cv::boundingRect(contour);
        double rectArea = (double)box.width * box.height;
        if (rectArea <= 0.0)
            continue;

        float aspect = (float)box.width / (float)std::max(1, box.height);
        if (aspect < 0.35f || aspect > 2.80f)
            continue;

        double fill = area / rectArea;
        if (fill < MIN_RECT_FILL)
            continue;

        double score = fill * area;
        if (score > bestScore)
        {
            bestScore = score;
            best.box = cv::Rect((int)std::round(box.x * invScale),
                                (int)std::round(box.y * invScale),
                                (int)std::round(box.width * invScale),
                                (int)std::round(box.height * invScale));
            best.center = {best.box.x + best.box.width / 2.0f,
                           best.box.y + best.box.height / 2.0f};
        }
    }

    if (bestScore <= 0.0)
        return false;

    out = best;
    return true;
}

void drawFps(cv::Mat &frame, double fps)
{
    char text[32];
    snprintf(text, sizeof(text), "FPS: %.1f", fps);

    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.65;
    const int thickness = 2;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Rect bg(8, 8, textSize.width + 18, textSize.height + baseline + 14);

    cv::rectangle(frame, bg, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
    cv::putText(frame, text, {bg.x + 9, bg.y + 9 + textSize.height},
                fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, cv::LINE_AA);
}

// ── 메인 ─────────────────────────────────────
int main()
{
    cv::setNumThreads(2);

    cv::VideoCapture cap;

    // USB 웹캠 우선 사용: 자동 검색 중 실제 프레임이 읽히는 장치를 선택
    if (!openUsbCamera(cap))
    {
        std::cerr << "[ERROR] USB 웹캠 실패: USB_CAMERA_INDEX 또는 /dev/video* 연결 상태 확인\n";
        return -1;
    }
    std::cout << "[INFO] Camera "
              << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ "
              << cap.get(cv::CAP_PROP_FPS) << "fps\n";

    cv::namedWindow("Tracking", cv::WINDOW_AUTOSIZE);

    // UART 초기화 (/dev/ttyACM0, 115200bps)
    int uartFd = uartOpen("/dev/ttyACM0", B115200);
    if (uartFd < 0)
        std::cerr << "[WARN] UART 없이 계속 실행\n";

    KalmanTracker kalman;
    cv::Mat frame;
    FrameGrabber grabber(cap);
    grabber.start();

    const cv::Point2f screen_center(CAM_W / 2.f, CAM_H / 2.f);
    int lostFrames = 0;
    int emptyFrames = 0;
    int frameCount = 0;
    double displayFps = 0.0;
    auto lastFrameTime = std::chrono::steady_clock::now();

    while (true)
    {
        if (!grabber.getLatest(frame))
        {
            emptyFrames++;
            if (emptyFrames == 1 || emptyFrames % 30 == 0)
                std::cerr << "[WARN] 캡처 스레드 프레임 대기 중: " << emptyFrames << "회\n";
            if ((cv::waitKey(10) & 0xFF) == 'q')
                break;
            continue;
        }
        emptyFrames = 0;
        frameCount++;

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFrameTime).count();
        lastFrameTime = now;
        if (elapsed > 0.0)
        {
            double instantFps = 1.0 / elapsed;
            displayFps = (displayFps <= 0.0)
                             ? instantFps
                             : (displayFps * FPS_SMOOTHING + instantFps * (1.0 - FPS_SMOOTHING));
        }

        // 노란색 마스크에서 공 객체 검출
        RectTarget rect;
        cv::Point2f kpt;
        bool runDetection = (frameCount % DETECT_INTERVAL_FRAMES) == 0;
        bool detected = runDetection && findYellowBall(frame, rect);

        // 칼만 필터
        if (detected)
        {
            kpt = kalman.update(rect.center);
            lostFrames = 0;
        }
        else
        {
            lostFrames++;
            kpt = (lostFrames <= 10) ? kalman.last() : cv::Point2f(-1, -1);
            if (lostFrames > 10)
                kalman.reset();
        }

        // ── 터미널 출력 ──
        if (kpt.x >= 0)
        {
            float dx = kpt.x - screen_center.x;
            float dy = kpt.y - screen_center.y;
            if (frameCount % PRINT_INTERVAL_FRAMES == 0)
            {
                printf("Kalman(%6.1f, %6.1f)  Center(%3.0f, %3.0f)  "
                       "dx=%+7.1f  dy=%+7.1f  dist=%.1f%s\n",
                       kpt.x, kpt.y,
                       screen_center.x, screen_center.y,
                       dx, dy,
                       std::sqrt(dx * dx + dy * dy),
                       detected ? "" : "  [PREDICT]");
            }

            // UART 전송: "dx,dy\n" (정수형으로 변환하여 전송)
            if (uartFd >= 0)
            {
                char pkt[32];
                int sendDy = (int)(dy * UART_OUTPUT_SCALE);
                int sendDx = (int)(dx * UART_OUTPUT_SCALE);
                int len = snprintf(pkt, sizeof(pkt), "%d,%d\n", sendDy, sendDx);
                write(uartFd, pkt, len);
            }
        }
        else
        {
            if (frameCount % PRINT_INTERVAL_FRAMES == 0)
                printf("-- 객체 없음 --\n");
        }

        // ── 화면 표시 ──
        if (detected)
        {
            int radius = std::max(rect.box.width, rect.box.height) / 2;
            cv::circle(frame, rect.center, radius, cv::Scalar(72, 192, 235), 2, cv::LINE_AA);
            cv::drawMarker(frame, rect.center, cv::Scalar(72, 192, 235),
                           cv::MARKER_CROSS, 10, 1, cv::LINE_AA);
        }

        if (kpt.x >= 0)
        {
            // 칼만 크로스헤어
            int cs = 14;
            cv::Scalar yellow(0, 255, 255);
            cv::line(frame, {(int)kpt.x - cs, (int)kpt.y}, {(int)kpt.x + cs, (int)kpt.y}, yellow, 2, cv::LINE_AA);
            cv::line(frame, {(int)kpt.x, (int)kpt.y - cs}, {(int)kpt.x, (int)kpt.y + cs}, yellow, 2, cv::LINE_AA);
            cv::circle(frame, kpt, cs + 2, yellow, 1, cv::LINE_AA);
        }

        // 화면 중앙 십자
        cv::drawMarker(frame, screen_center, cv::Scalar(200, 200, 200),
                       cv::MARKER_CROSS, 20, 1, cv::LINE_AA);

        drawFps(frame, displayFps);
        cv::imshow("Tracking", frame);
        if ((cv::waitKey(1) & 0xFF) == 'q')
            break;
    }

    grabber.stop();
    if (uartFd >= 0)
        close(uartFd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
