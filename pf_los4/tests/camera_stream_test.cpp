#include "camera_stream.hpp"
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    const std::string file = argv[1];
    cv::VideoWriter writer(file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           10.0, cv::Size(64, 48));
    if (!writer.isOpened()) return 2;
    for (int i = 0; i < 5; ++i) {
        cv::Mat image(48, 64, CV_8UC3, cv::Scalar(i * 30, i * 30, i * 30));
        writer.write(image);
    }
    writer.release();
    std::mutex mutex;
    std::condition_variable ready;
    std::vector<std::shared_ptr<const CameraFrame>> frames;
    const auto started = std::chrono::steady_clock::now();
    CameraStream stream(file, [&](auto frame) {
        std::lock_guard<std::mutex> lock(mutex);
        frames.push_back(std::move(frame));
        ready.notify_one();
    });
    bool received = false;
    {
        std::unique_lock<std::mutex> lock(mutex);
        received = ready.wait_for(lock, std::chrono::seconds(5), [&] { return frames.size() >= 5; });
    }
    stream.stop();
    if (!received) return 3;
    for (std::size_t i = 0; i < 5; ++i) {
        if (frames[i]->sequence != i + 1 || frames[i]->received < started ||
            frames[i]->image.cols != 64 || frames[i]->image.rows != 48 ||
            frames[i]->image.channels() != 3) return 4;
    }
    // A later decode must not overwrite the pixels in an earlier snapshot.
    if (frames[0]->image.at<cv::Vec3b>(0, 0)[0] > 5 ||
        frames[4]->image.at<cv::Vec3b>(0, 0)[0] < 110) return 5;
    CameraStream disabled("", [](auto) {});
    disabled.stop();
    std::filesystem::remove(file);
    return 0;
}
