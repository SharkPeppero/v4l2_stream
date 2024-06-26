//
// Created by xu on 24-4-26.
//
#include "v4l2_stream.h"
#include <utility>

// ***************************** V4L2Stream  ***************************** //
// V4L2Stream初始化
V4L2Stream::V4L2Stream(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {

    std::string cfg_file = std::string(ROOT_DIR) + "install/v4l2_stream/share/v4l2_stream/config/usb_circle_camera.yaml";
    log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "ymal文件路径： %s\n", cfg_file.c_str());

    try {
        // 加载配置文件
        YAML::Node config = YAML::LoadFile(cfg_file);

        // 遍历相机节点
        auto camera_nodes = config["cameras"]; // 第一个节点
        for (const auto& camera_node: camera_nodes) {
            auto open = camera_node["open"].as<bool>();
            auto device = camera_node["device"].as<std::string>();
            auto format = camera_node["format"].as<int>();
            auto width = camera_node["width"].as<int>();
            auto height = camera_node["height"].as<int>();
            auto fps = camera_node["fps"].as<int>();
            auto pub_topic = camera_node["pub_topic"].as<std::string>();

            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "");
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "open: %s", (open == true ? std::string("true").c_str(): std::string("false").c_str()));
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "device: %s", device.c_str());
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "format: %s", std::to_string(format).c_str());
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "width: %s", std::to_string(width).c_str());
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "height: %s", std::to_string(height).c_str());
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "fps: %s", std::to_string(fps).c_str());
            log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "pub_topic: %s", pub_topic.c_str());

            if(!open){
                log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s is not open.", device.c_str());
                continue;
            }

            // 创建发布对象
            auto pub_object = node_->create_publisher<ROS_IMAGE_TYPE>(pub_topic, rclcpp::SystemDefaultsQoS());

            // 创建相机属性对象
            CameraInfo cameraInfo(open, device, format, width, height, fps, pub_topic);
            auto tmp = std::make_shared<V4L2StreamUtil>(cameraInfo, pub_object);
            v4l2StreamUtil_vec_.push_back(tmp);
        }
    } catch (const YAML::BadFile &bf_err) {
        RCLCPP_ERROR(node_->get_logger(), "failed to load \"%s\"!", cfg_file.c_str());
        return;
    } catch (const YAML::BadConversion &bc_err) {
        RCLCPP_ERROR(node_->get_logger(), "conversion error in \"%s\"!", cfg_file.c_str());
        return;
    } catch (const YAML::BadSubscript &bs_err) {
        RCLCPP_ERROR(node_->get_logger(), "sub-type error in \"%s\"!", cfg_file.c_str());
        return;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "other error took place in CameraStreamManager initialization!");
        return;
    }
}

// ***************************** V4L2StreamUtil  ***************************** //

// v4l2设置相机参数进入取流线程
V4L2StreamUtil::V4L2StreamUtil(const CameraInfo &cameraInfo, rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_object)  : cameraInfo_(cameraInfo),
                                                                                                                         pub_image_(std::move(pub_object)) {

    // 打印参数
    // 打印数据
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "open: %s", (cameraInfo_.open_ == true ? std::string("true").c_str(): std::string("false").c_str()));
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "device: %s", cameraInfo_.device_.c_str());
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "format: %s", std::to_string(cameraInfo_.format_).c_str());
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "width: %s", std::to_string(cameraInfo_.width_).c_str());
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "height: %s", std::to_string(cameraInfo_.height_).c_str());
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "fps: %s", std::to_string(cameraInfo_.fps_).c_str());
    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "pub_topic: %s", cameraInfo_.pub_topic_.c_str());

    // 设置v4l2相机取流参数
    v4L2Capture_.set_device(cameraInfo.device_);
    v4L2Capture_.set_resolution(cameraInfo.width_, cameraInfo.height_);
    v4L2Capture_.set_frame_rate(cameraInfo.fps_);

    if(v4L2Capture_.init(cameraInfo_.format_) == 0){
        log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully init v4l2 camera.", cameraInfo_.device_.c_str());
    }else{
        log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully init v4l2 camera.", cameraInfo_.device_.c_str());
    }

    if(v4L2Capture_.camera_capture_start() == 0){
        log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully start v4l2 camera.", cameraInfo_.device_.c_str());
    }else{
        log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully start v4l2 camera.", cameraInfo_.device_.c_str());
    }

    log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s loopUnit...", cameraInfo_.device_.c_str());
    thread_ = std::thread(&V4L2StreamUtil::loopUnit, this);
    thread_.detach();
}

// v4l2取流发布
void V4L2StreamUtil::loopUnit() {
    while (rclcpp::ok() && !cancel_) {
        auto t1 = std::chrono::steady_clock::now();
        cv::Mat rgbImage;

        buffer_ = v4L2Capture_.get_one_frame(); // 获取数据

        if(cameraInfo_.format_ == 0){
            // Decode YUYV
            cv::Mat img = cv::Mat(cv::Size(cameraInfo_.width_, cameraInfo_.height_), CV_8UC2, (uint8_t *) buffer_.data);
            cv::cvtColor(img, rgbImage, cv::COLOR_YUV2RGB_YVYU);
        }else if(cameraInfo_.format_ == 1){
            std::vector<uint8_t> mjpg_data((uint8_t *)buffer_.data, (uint8_t *)buffer_.data + buffer_.length);
            rgbImage = cv::imdecode(mjpg_data, cv::IMREAD_COLOR);
        }else if(cameraInfo_.format_ == 2){

        }else{

        }

        v4L2Capture_.clear_one_frame(); // 清除缓存

        pub_image_->publish(ROSImageFromMat(rgbImage, getRosTime()));
    }
}

// ***************************** V4L2StreamUtil 工具部分 ***************************** //
builtin_interfaces::msg::Time V4L2StreamUtil::getRosTime() {
    struct timespec thread_cpu_time{};
    clock_gettime(CLOCK_REALTIME, &thread_cpu_time);
    double cur_timestamp = thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
    int32_t seconds = static_cast<int32_t>(std::floor(cur_timestamp));
    uint32_t nanosec = static_cast<int32_t>((cur_timestamp - seconds) * 1e9);
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = seconds;
    msg_time.nanosec = nanosec;
    return msg_time;
}

ROS_IMAGE_TYPE V4L2StreamUtil::ROSImageFromMat(const cv::Mat &mat, const builtin_interfaces::msg::Time &msg_time) {
    ROS_IMAGE_TYPE msg;
    try {
        // 将OpenCV图像转换为ROS图像消息
        cv_bridge::CvImage cv_image;
        cv_image.image = mat;
        if (mat.channels() == 1) {
            cv_image.encoding = "mono8";
        } else {
            cv_image.encoding = "bgr8";
        }
        msg = *cv_image.toImageMsg();
    }
    catch (cv_bridge::Exception &e) {
        // 如果无法将OpenCV图像转换为ROS图像消息，则会抛出异常
        printf("Could not convert from 'bgr8' to '%s'.", msg.encoding.c_str());
    }
    msg.header.stamp = msg_time;
    return msg;
}

cv::Mat V4L2StreamUtil::MatFromROSImage(const ROS_IMAGE_TYPE &msg) {
    cv::Mat transformed_image;
    try {
        // 将ROS图像消息转换为OpenCV图像格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 在这里可以对OpenCV图像进行操作
        transformed_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e) {
        // 如果无法将ROS图像消息转换为OpenCV图像格式，则会抛出异常
        printf("Could not convert from '%s' to 'bgr8'.", msg.encoding.c_str());
    }

    return transformed_image;
}

void V4L2StreamUtil::yuyv422_to_mat(const uint8_t *yuyv_data, int width, int height, cv::Mat &mat) {
    mat.create(height, width, CV_8UC3);
    int yuyv_index = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 2) {
            int y1 = yuyv_data[yuyv_index++];
            int u = yuyv_data[yuyv_index++];
            int y2 = yuyv_data[yuyv_index++];
            int v = yuyv_data[yuyv_index++];

            int r1 = y1 + 1.13983 * (v - 128);
            int g1 = y1 - 0.39465 * (u - 128) - 0.58060 * (v - 128);
            int b1 = y1 + 2.03211 * (u - 128);

            int r2 = y2 + 1.13983 * (v - 128);
            int g2 = y2 - 0.39465 * (u - 128) - 0.58060 * (v - 128);
            int b2 = y2 + 2.03211 * (u - 128);

            mat.at<cv::Vec3b>(y, x)[0] = cv::saturate_cast<unsigned char>(b1);
            mat.at<cv::Vec3b>(y, x)[1] = cv::saturate_cast<unsigned char>(g1);
            mat.at<cv::Vec3b>(y, x)[2] = cv::saturate_cast<unsigned char>(r1);

            mat.at<cv::Vec3b>(y, x + 1)[0] = cv::saturate_cast<unsigned char>(b2);
            mat.at<cv::Vec3b>(y, x + 1)[1] = cv::saturate_cast<unsigned char>(g2);
            mat.at<cv::Vec3b>(y, x + 1)[2] = cv::saturate_cast<unsigned char>(r2);
        }
    }
}

void V4L2StreamUtil::YUY2toI420(int inWidth, int inHeight, uint8_t *pSrc, uint8_t *pDest) {
    int i, j;
    //首先对I420的数据整体布局指定
    uint8_t *u = pDest + (inWidth * inHeight);
    uint8_t *v = u + (inWidth * inHeight) / 4;

    for (i = 0; i < inHeight / 2; i++) {
        /*采取的策略是:在外层循环里面，取两个相邻的行*/
        uint8_t *src_l1 = pSrc + inWidth * 2 * 2 * i;//因为4:2:2的原因，所以占用内存，相当一个像素占2个字节，2个像素合成4个字节
        uint8_t *src_l2 = src_l1 + inWidth * 2;//YUY2的偶数行下一行
        uint8_t *y_l1 = pDest + inWidth * 2 * i;//偶数行
        uint8_t *y_l2 = y_l1 + inWidth;//偶数行的下一行
        for (j = 0; j < inWidth / 2; j++)//内层循环
        {
            // two pels in one go//一次合成两个像素
            //偶数行，取完整像素;Y,U,V;偶数行的下一行，只取Y
            *y_l1++ = src_l1[0];//Y
            *u++ = src_l1[1];//U
            *y_l1++ = src_l1[2];//Y
            *v++ = src_l1[3];//V
            //这里只有取Y
            *y_l2++ = src_l2[0];
            *y_l2++ = src_l2[2];
            //YUY2,4个像素为一组
            src_l1 += 4;
            src_l2 += 4;
        }
    }
}




// ***************************** V4L2Stream 工具部分 ***************************** //
// 参数解析创建相机信息对象
std::shared_ptr<CameraInfo> V4L2Stream::parseCameraNodeToCI(bool &open, const YAML::Node &camera_node) {
    std::shared_ptr<CameraInfo> tmp = std::make_shared<CameraInfo>();
    tmp->device_ = camera_node["device"].as<std::string>();
    tmp->format_ = camera_node["format"].as<int>();
    tmp->width_ = camera_node["width"].as<int>();
    tmp->height_ = camera_node["height"].as<int>();
    tmp->fps_ = camera_node["fps"].as<int>();
    tmp->pub_topic_ = camera_node["pub_topic"].as<std::string>();
    return tmp;
}

