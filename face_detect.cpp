/*
 * color_detect.cpp — 색상 인식 + 칼만 필터
 * 빌드: make  /  실행: make run
 * 종료: Q 또는 ESC
 */

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static constexpr int CAM_W = 640;
static constexpr int CAM_H = 480;
static constexpr int CAM_FPS = 30;

// ── 칼만 필터 ────────────────────────────────
// 상태 [cx, cy, vx, vy] / 측정 [cx, cy]
class KalmanTracker
{
public:
    KalmanTracker() : kf_(4, 2, 0), initialized_(false)
    {
        kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar(3e-3));    // Q 낮춤 → 더 부드럽게
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(8.0)); // R 높임 → 측정 노이즈 무시
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

    cv::Point2f predict()
    {
        if (!initialized_)
            return {-1, -1};
        cv::Mat p = kf_.predict();
        return {p.at<float>(0), p.at<float>(1)};
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

// ── GStreamer 파이프라인 ──────────────────────
std::string gstPipe()
{
    return "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 ! "
           "videoconvert ! video/x-raw,format=BGR ! "
           "appsink drop=true max-buffers=1";
}

// ── 가장 큰 Blob 검출 ────────────────────────
struct Blob
{
    cv::Rect box;
    cv::Point2f center;
};

bool findLargest(const cv::Mat &mask, Blob &out)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best = 800.0;
    bool found = false;
    for (auto &c : contours)
    {
        double a = cv::contourArea(c);
        if (a < best)
            continue;
        best = a;
        found = true;
        auto m = cv::moments(c);
        out.box = cv::boundingRect(c);
        out.center = (m.m00 > 0)
                         ? cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00))
                         : cv::Point2f(out.box.x + out.box.width / 2.f,
                                       out.box.y + out.box.height / 2.f);
    }
    return found;
}

// ── 메인 ─────────────────────────────────────
int main()
{
    // 빨강: 0~15° + 160~180° (두 범위 합산)
    cv::Scalar lo1(0, 80, 60), hi1(15, 255, 255);
    cv::Scalar lo2(160, 80, 60), hi2(180, 255, 255);

    cv::VideoCapture cap;
    cap.open(gstPipe(), cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "[WARN] GStreamer 실패 → /dev/video0 시도\n";
        cap.open(0);
        if (!cap.isOpened())
        {
            std::cerr << "[ERROR] 카메라 실패\n";
            return -1;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, CAM_W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_H);
        cap.set(cv::CAP_PROP_FPS, CAM_FPS);
    }

    cv::namedWindow("Tracking", cv::WINDOW_AUTOSIZE);

    // UART 초기화 (/dev/ttyACM0, 115200bps)
    int uartFd = uartOpen("/dev/ttyACM0", B115200);
    if (uartFd < 0)
        std::cerr << "[WARN] UART 없이 계속 실행\n";

    KalmanTracker kalman;
    cv::Mat frame, hsv, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});

    const cv::Point2f screen_center(CAM_W / 2.f, CAM_H / 2.f);
    int lostFrames = 0;

    while (true)
    {
        if (!cap.read(frame) || frame.empty())
        {
            cv::waitKey(10);
            continue;
        }

        // HSV 변환 + 마스크
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask2;
        cv::inRange(hsv, lo1, hi1, mask);
        cv::inRange(hsv, lo2, hi2, mask2);
        cv::bitwise_or(mask, mask2, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 칼만 필터
        Blob blob;
        cv::Point2f kpt;
        bool detected = findLargest(mask, blob);

        if (detected)
        {
            kpt = kalman.update(blob.center);
            lostFrames = 0;
        }
        else
        {
            lostFrames++;
            kpt = (lostFrames <= 30) ? kalman.predict() : cv::Point2f(-1, -1);
            if (lostFrames > 30)
                kalman.reset();
        }

        // ── 터미널 출력 ──
        if (kpt.x >= 0)
        {
            float dx = kpt.x - screen_center.x;
            float dy = kpt.y - screen_center.y;
            printf("Kalman(%6.1f, %6.1f)  Center(%3.0f, %3.0f)  "
                   "dx=%+7.1f  dy=%+7.1f  dist=%.1f%s\n",
                   kpt.x, kpt.y,
                   screen_center.x, screen_center.y,
                   dx, dy,
                   std::sqrt(dx * dx + dy * dy),
                   detected ? "" : "  [PREDICT]");

// UART 전송: "dx,dy\n" (정수형으로 변환하여 전송)
            if (uartFd >= 0)
            {
                char pkt[32];
                // %+.1f를 %d로 바꾸고, 변수 앞에 (int)를 붙여줍니다.
                int len = snprintf(pkt, sizeof(pkt), "%d,%d\n", (int)dy, (int)dx);
                write(uartFd, pkt, len);
            }
        }
        else
        {
            printf("-- 객체 없음 --\n");
        }

        // ── 화면 표시 ──
        if (detected)
            cv::rectangle(frame, blob.box, cv::Scalar(0, 200, 0), 1, cv::LINE_AA);

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

        cv::imshow("Tracking", frame);
        if ((cv::waitKey(1) & 0xFF) == 'q')
            break;
    }

    if (uartFd >= 0)
        close(uartFd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
