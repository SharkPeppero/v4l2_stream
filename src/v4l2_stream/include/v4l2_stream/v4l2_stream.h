//
// Created by xu on 24-4-26.
//

#ifndef V4L2_STREAM_V4L2_STREAM_H
#define V4L2_STREAM_V4L2_STREAM_H

#include "deque"
#include "memory"
#include <utility>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>

#include "thread_pool/thread_pool.h"
#include "time_utils/time_utils.h"

#include "mix_helper/mix_helper.h"
#include "v4l2_capture//v4l2_capture.h"

#include "logger/logger.h"

using ROS_IMAGE_TYPE = sensor_msgs::msg::Image;

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
class V4L2StreamUtil{
public:
    explicit V4L2StreamUtil(const CameraInfo &cameraInfo,
                            rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_object);

    void loopUnit();

    // ***************************** 工具部分 ***************************** //
    // YUV422转Mat
    void yuyv422_to_mat(const uint8_t * yuyv_data, int width, int height, cv::Mat& mat);

    // YUV2(YUYV)转I420
    void YUY2toI420(int inWidth, int inHeight, uint8_t *pSrc, uint8_t *pDest);

    // cv::Mat 转 ROS消息
    static ROS_IMAGE_TYPE ROSImageFromMat(const cv::Mat &mat, const builtin_interfaces::msg::Time &msg_time);

    // ROS 转 cv::Mat
    static cv::Mat MatFromROSImage(const ROS_IMAGE_TYPE &msg);

    // 获取系统时间
    static builtin_interfaces::msg::Time getRosTime();

private:
    CameraInfo cameraInfo_;

    V4L2Capture v4L2Capture_;
    Buffer buffer_{};

    std::atomic<bool> cancel_ {false}; // 原子变量，可以确保操作是不会被其他线程影响
    std::thread thread_;
    cv::Mat img_;

    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_image_;
};

// 根据yaml文件设置相机参数，初始化相机对象
class V4L2Stream{
public:
    explicit V4L2Stream(rclcpp::Node::SharedPtr node);

    ~V4L2Stream() = default;

private:

    // ***************************** 工具部分 ***************************** //
    // 参数解析
    static std::shared_ptr<CameraInfo> parseCameraNodeToCI(bool &open, const YAML::Node &camera_node);

private:
    rclcpp::Node::SharedPtr node_;

    // v4l2取流工具对象队列
    std::vector<std::shared_ptr<V4L2StreamUtil>> v4l2StreamUtil_vec_;

    // 定时器发布线程
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr> pub_vec_;
};

#endif //V4L2_STREAM_V4L2_STREAM_H
