//
// Created by wting on 24-7-2.
//

#ifndef V4L2_STREAM_V4L2_STREAM_OBJECT_H
#define V4L2_STREAM_V4L2_STREAM_OBJECT_H

#include "deque"
#include "memory"
#include <utility>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>

#include "thread_pool/thread_pool.h"
#include "time_utils/time_utils.h"

#include "mix_helper/mix_helper.h"
#include "v4l2_capture//v4l2_capture.h"

#include "logger/logger.h"

struct ImageWithTimestamp{
    ImageWithTimestamp(cv::Mat image, double timestamp){
        image_ = image;
        timestamp_ = timestamp;
    }

    cv::Mat image_;
    double timestamp_;
};

struct CameraInfo {
    CameraInfo() = default;

    CameraInfo(bool open,
               const std::string &device,
               int format,
               int width,
               int height,
               int fps,
               const std::string &pub_topic) {
        open_ = open;
        device_ = device;
        format_ = format;
        width_ = width;
        height_ = height;
        fps_ = fps;
        pub_topic_ = pub_topic;
    }

    CameraInfo(const CameraInfo &other) {
        open_ = other.open_;
        device_ = other.device_;
        format_ = other.format_;
        width_ = other.width_;
        height_ = other.height_;
        fps_ = other.fps_;
        pub_topic_ = other.pub_topic_;
    }

    CameraInfo &operator=(const CameraInfo &other) = default;

    ~CameraInfo() = default;

    bool open_{};
    std::string device_;
    int format_{};
    int width_{};
    int height_{};
    int fps_{};
    std::string pub_topic_;
};

// v4l2相机对象类
class V4L2StreamObject{
public:
    explicit V4L2StreamObject(const CameraInfo &cameraInfo);

    void loopUnit();

    void setImageWithTimeStamp(const std::shared_ptr<ImageWithTimestamp> &image_timestamp_ptr) {
        std::lock_guard<std::mutex> lock(image_timestamp_mtx_);
        image_timestamp_ptr_ = image_timestamp_ptr;
    }

    void getImageWithTimeStamp(std::shared_ptr<ImageWithTimestamp>& image_timestamp_ptr){
        std::lock_guard<std::mutex> lock(image_timestamp_mtx_);
        image_timestamp_ptr = image_timestamp_ptr_;
    }

    // ***************************** 工具部分 ***************************** //
    // 获取系统时间
    static double getSystemTime();

    // YUV422转Mat
    void yuyv422_to_mat(const uint8_t * yuyv_data, int width, int height, cv::Mat& mat);

    // YUV2(YUYV)转I420
    void YUY2toI420(int inWidth, int inHeight, uint8_t *pSrc, uint8_t *pDest);

    CameraInfo getCameraInfo(){
        return cameraInfo_;
    }

private:
    CameraInfo cameraInfo_;

    V4L2Capture v4L2Capture_;
    Buffer buffer_{};

    std::atomic<bool> cancel_ {false}; // 原子变量，可以确保操作是不会被其他线程影响
    std::thread thread_;

    std::mutex image_timestamp_mtx_;
    std::shared_ptr<ImageWithTimestamp> image_timestamp_ptr_ = nullptr;
};

#endif //V4L2_STREAM_V4L2_STREAM_OBJECT_H
