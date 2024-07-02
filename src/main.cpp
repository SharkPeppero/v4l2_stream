//
// Created by xu on 24-4-26.
//

#include "v4l2_stream_object/v4l2_stream_object.h"

std::vector<std::shared_ptr<V4L2StreamObject> > v4l2_stream_object_vec;
void InitCamera(std::string cfg_file){
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

            // 创建相机属性对象
            CameraInfo cameraInfo(open, device, format, width, height, fps, pub_topic);
            auto tmp = std::make_shared<V4L2StreamObject>(cameraInfo);
            v4l2_stream_object_vec.push_back(tmp);
        }
    } catch (const YAML::BadFile &bf_err) {
        log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "failed to load \"%s\"!", cfg_file.c_str());
        return;
    } catch (const YAML::BadConversion &bc_err) {
        log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "conversion error in \"%s\"!", cfg_file.c_str());
        return;
    } catch (const YAML::BadSubscript &bs_err) {
        log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "sub-type error in \"%s\"!", cfg_file.c_str());
        return;
    } catch (...) {
        log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "other error took place in CameraStreamManager initialization!");
        return;
    }
}

int main(int argc, char *argv[]) {
    // 读取配置文件，初始化相机数据
    std::string cfg_file = std::string(ROOT_DIR) + "src/v4l2_stream/config/usb_circle_camera.yaml";
    log__save("Params", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "Yaml文件路径： %s\n", cfg_file.c_str());

    // 初始化相机对象
    InitCamera(cfg_file);


    std::unordered_map<std::string, double> previous_timestamps;
    std::unordered_map<std::string, double> frame_rates;

    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (v4l2_stream_object_vec.empty()) {
            continue;
        }

        for (const auto &cameraObject : v4l2_stream_object_vec) {
            std::shared_ptr<ImageWithTimestamp> image_timestamp_ptr = nullptr;
            cameraObject->getImageWithTimeStamp(image_timestamp_ptr);
            if (image_timestamp_ptr != nullptr) {

                // 计算帧率
                std::string device_name = cameraObject->getCameraInfo().device_;
                double current_timestamp = image_timestamp_ptr->timestamp_;
                if (previous_timestamps.find(device_name) != previous_timestamps.end()) {
                    double previous_timestamp = previous_timestamps[device_name];
                    double time_diff_ns = current_timestamp - previous_timestamp;
                    if (time_diff_ns > 0) {
                        double frame_rate = 1.0 / time_diff_ns; // 1e9 是将纳秒转换为秒
                        frame_rates[device_name] = frame_rate;
                    }
                }
                previous_timestamps[device_name] = current_timestamp;

                // 在图像上显示时间戳和帧率
                cv::putText(image_timestamp_ptr->image_,
                            "Timestamp: " + std::to_string(current_timestamp),
                            cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0,
                            cv::Scalar(255, 255, 255),
                            2);
                if (frame_rates.find(device_name) != frame_rates.end()) {
                    cv::putText(image_timestamp_ptr->image_,
                                "FPS: " + std::to_string(frame_rates[device_name]),
                                cv::Point(10, 60),
                                cv::FONT_HERSHEY_SIMPLEX,
                                1.0,
                                cv::Scalar(255, 255, 255),
                                2);
                }
                cv::imshow(device_name, image_timestamp_ptr->image_);
            }
        }

        auto key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();

}