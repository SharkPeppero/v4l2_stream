//
// Created by xu on 24-4-26.
//

#include "v4l2_stream/v4l2_stream.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("v4l2_stream");

    auto V4L2StreamNode = std::make_shared<V4L2Stream>(node);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}