#ifdef ENABLE_ROS2

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

namespace {

const std::string node_name{"video_publisher"};
const std::string topic_name{"aruco_camera/image"};

} // namespace

int main(int argc, char *argv[]) {
    auto args{rclcpp::init_and_remove_ros_arguments(argc, argv)};
    if (args.size() < 2) {
        std::cerr << "Usage: " << argv[0] << " [camera|video_file] [camera_device]"
                  << std::endl;
        std::cerr << "Note: camera_device, width and height parameters are required for camera input"
                  << std::endl;
        return EXIT_FAILURE;
    }

    const std::string &video_input{args[1]};
    bool use_camera = (video_input == "camera");

    auto node = rclcpp::Node::make_shared(node_name);
    image_transport::ImageTransport it{node};
    auto publisher = it.advertise(topic_name, 1);

    cv::VideoCapture capture;
    if (use_camera) {
        if (args.size() != 3) {
            std::cerr << "Error: Camera input requires all parameters: camera_device width height"
                      << std::endl;
            return EXIT_FAILURE;
        }

        std::string camera_device = args[2];

        // Open the camera using the V4L2 backend.
        capture.open(camera_device, cv::CAP_V4L2);
        if (!capture.isOpened()) {
            std::cerr << "Error: Could not open camera device " << camera_device
                      << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Using camera device: " << camera_device << std::endl;

        // Set camera properties.
        capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        capture.set(cv::CAP_PROP_FPS, 30);
    } else {
        // Open the video file.
        capture.open(video_input);
        if (!capture.isOpened()) {
            std::cerr << "Error: Could not open video file: " << video_input
                      << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Using video file: " << video_input << std::endl;
    }

    cv::Mat image;
    if (use_camera) {
        // 30 Fps seems to be what jetson can handle
        const int fps = 30;
        auto frame_duration = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));

        while (rclcpp::ok()) {
            if (!capture.read(image)) {
                std::cerr << "Error: Couldn't capture frame from camera"
                          << std::endl;
                break;
            }

            std_msgs::msg::Header header;
            header.stamp = node->now();
            header.frame_id = "camera_frame";

            auto message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, image)
                      .toImageMsg();
            publisher.publish(message);

            rclcpp::spin_some(node);
            std::this_thread::sleep_for(frame_duration);
        }
    } else {
        // For video file playback, synchronize the publishing using the video's
        // timestamps.
        rclcpp::Clock clock{};
        const rclcpp::Time start_time{clock.now()};

        while (rclcpp::ok()) {
            if (!capture.read(image)) {
                std::cerr << "Reached end of video or error reading frame."
                          << std::endl;
                break;
            }

            // Use the video's timestamp (in milliseconds) to compute the delay.
            double pos_msec = capture.get(cv::CAP_PROP_POS_MSEC);
            rclcpp::Duration target_after_start{std::chrono::nanoseconds(
                static_cast<long long>(pos_msec * 1e6))};
            clock.sleep_until(start_time + target_after_start);

            std_msgs::msg::Header header;
            header.stamp = node->now();
            header.frame_id = "video_frame";

            auto message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image)
                              .toImageMsg();
            publisher.publish(message);

            rclcpp::spin_some(node);
        }
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

#endif // ENABLE_ROS2
