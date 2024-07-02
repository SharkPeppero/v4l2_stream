//
// Created by wting on 24-7-2.
//

#include "v4l2_stream_object.h"


// ***************************** V4L2StreamObject  ***************************** //

// v4l2设置相机参数进入取流线程
V4L2StreamObject::V4L2StreamObject(const CameraInfo &cameraInfo) : cameraInfo_(cameraInfo) {

    // 打印数据
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "open: %s", (cameraInfo_.open_ == true ? std::string("true").c_str(): std::string("false").c_str()));
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "device: %s", cameraInfo_.device_.c_str());
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "format: %s", std::to_string(cameraInfo_.format_).c_str());
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "width: %s", std::to_string(cameraInfo_.width_).c_str());
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "height: %s", std::to_string(cameraInfo_.height_).c_str());
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "fps: %s", std::to_string(cameraInfo_.fps_).c_str());
    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "pub_topic: %s", cameraInfo_.pub_topic_.c_str());

    // 设置v4l2相机取流参数
    v4L2Capture_.set_device(cameraInfo.device_);
    v4L2Capture_.set_resolution(cameraInfo.width_, cameraInfo.height_);
    v4L2Capture_.set_frame_rate(cameraInfo.fps_);

    if(v4L2Capture_.init(cameraInfo_.format_) == 0){
        log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully init v4l2 camera.", cameraInfo_.device_.c_str());
    }else{
        log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully init v4l2 camera.", cameraInfo_.device_.c_str());
    }

    if(v4L2Capture_.camera_capture_start() == 0){
        log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully start v4l2 camera.", cameraInfo_.device_.c_str());
    }else{
        log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully start v4l2 camera.", cameraInfo_.device_.c_str());
    }

    log__save("V4L2StreamObject", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s loopUnit...", cameraInfo_.device_.c_str());
    thread_ = std::thread(&V4L2StreamObject::loopUnit, this);
    thread_.detach();
}

// v4l2取流发布
void V4L2StreamObject::loopUnit() {
    while (!cancel_) {
        auto t1 = std::chrono::steady_clock::now();
        cv::Mat rgbImage;

        buffer_ = v4L2Capture_.get_one_frame(); // 获取数据

        if(cameraInfo_.format_ == 0){
            // Decode YUYV
            cv::Mat img = cv::Mat(cv::Size(cameraInfo_.width_, cameraInfo_.height_), CV_8UC2, (uint8_t *) buffer_.data);
            cv::cvtColor(img, rgbImage, cv::COLOR_YUV2RGB_YVYU);
        }else if(cameraInfo_.format_ == 1){
            // Decode MJPG
            std::vector<uint8_t> mjpg_data((uint8_t *)buffer_.data, (uint8_t *)buffer_.data + buffer_.length);
            rgbImage = cv::imdecode(mjpg_data, cv::IMREAD_COLOR);
        }else if(cameraInfo_.format_ == 2){

        }else{

        }
        ImageWithTimestamp imageWithTimestamp(rgbImage, getSystemTime());
        setImageWithTimeStamp(std::make_shared<ImageWithTimestamp>(imageWithTimestamp));

        v4L2Capture_.clear_one_frame(); // 清除缓存
    }
}

// 获取系统时间戳
double V4L2StreamObject::getSystemTime() {
    struct timespec thread_cpu_time{};
    clock_gettime(CLOCK_REALTIME, &thread_cpu_time);
    double cur_timestamp = thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
    return cur_timestamp;
}

// 数据格式转换
void V4L2StreamObject::yuyv422_to_mat(const uint8_t *yuyv_data, int width, int height, cv::Mat &mat) {
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

// 数据格式转换
void V4L2StreamObject::YUY2toI420(int inWidth, int inHeight, uint8_t *pSrc, uint8_t *pDest) {
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