#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

struct CameraFrame {
    cv::Mat image; // Decoded BGR image owned by this frame.
    std::uint64_t sequence{0};
    std::chrono::steady_clock::time_point received;
};

// Video arrives over its own IP connection. The callback shares an immutable
// latest frame with the MAVSDK telemetry cache, avoiding copies under its lock.
class CameraStream {
public:
    using Callback = std::function<void(std::shared_ptr<const CameraFrame>)>;
    CameraStream(std::string url, Callback callback)
        : url_(std::move(url)), callback_(std::move(callback)) {
        if (!url_.empty()) worker_ = std::thread([this] { receive(); });
    }
    ~CameraStream() { stop(); }
    CameraStream(const CameraStream&) = delete;
    CameraStream& operator=(const CameraStream&) = delete;
    void stop() {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
    }
private:
    void retry_delay() {
        for (int i = 0; i < 20 && running_.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    void receive() {
        std::uint64_t sequence = 0;
        while (running_.load()) {
            try {
                cv::VideoCapture capture;
                const std::vector<int> options{
                    cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
                    cv::CAP_PROP_READ_TIMEOUT_MSEC, 1000};
                if (!capture.open(url_, cv::CAP_FFMPEG, options)) {
                    std::cerr << "Camera stream unavailable; retrying in 2 seconds.\n";
                    retry_delay();
                    continue;
                }
                std::cerr << "Camera stream connected.\n";
                while (running_.load()) {
                    cv::Mat decoded;
                    if (!capture.read(decoded) || decoded.empty()) break;
                    if (!running_.load()) break;
                    auto frame = std::make_shared<CameraFrame>();
                    frame->received = std::chrono::steady_clock::now();
                    frame->sequence = ++sequence;
                    frame->image = decoded.clone();
                    callback_(std::move(frame));
                }
                capture.release();
            } catch (const cv::Exception& error) {
                std::cerr << "Camera decoder error: " << error.what() << '\n';
            }
            if (running_.load()) {
                std::cerr << "Camera stream ended; retrying in 2 seconds.\n";
                retry_delay();
            }
        }
    }
    std::string url_;
    Callback callback_;
    std::atomic<bool> running_{true};
    std::thread worker_;
};
