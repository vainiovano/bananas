#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#ifdef ENABLE_ROS2
#include <algorithm>
#include <memory>
#include <sstream>
#else // ENABLE_ROS2
#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>
#endif // ENABLE_ROS2

#ifdef ENABLE_ROS2
#include <gsl/assert>
#include <gsl/pointers>
#endif // ENABLE_ROS2

#include <nlohmann/json.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#ifndef ENABLE_ROS2
#include <opencv2/videoio.hpp>
#endif // ENABLE_ROS2

#ifdef ENABLE_ROS2
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif // ENABLE_ROS2

#include <mavsdk/component_type.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/concrete_board.h>
#include <bananas_aruco/mavlink.h>
#include <bananas_aruco/visualization/visualizer.h>
#include <bananas_aruco/world.h>

namespace {

const char *const about{"Find camera and box locations from a video"};
const char *const keys{
    "{env     | <none> | JSON file describing the static environment }"
    "{boards  | <none> | JSON file containing the board descriptions }"
    "{camera  | <none> | JSON file containing the camera information }"
    "{mavlink |        | Mavlink URL }"
#ifndef ENABLE_ROS2
    "{@infile | <none> | Input video }"
    "{vo      |        | Video output file }"
#endif // ENABLE_ROS2
};

struct CameraConfiguration {
    float focal_length_x{};
    float focal_length_y{};
    float optical_center_x{};
    float optical_center_y{};
    std::vector<float> distortion_coefficients{};
    affine_rotation::AffineRotation camera_to_drone{};
};

// Mark to_json as potentially unused to make clang happy. This is ugly, but it
// works.
[[maybe_unused]] NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    CameraConfiguration, focal_length_x, focal_length_y, optical_center_x,
    optical_center_y, distortion_coefficients, camera_to_drone);

/// Returns true if the user wants to exit the application. Handles pausing and
/// blocks until the user unpauses.
auto handle_keys() -> bool {
    bool paused{false};
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-do-while)
    do {
        const int key{cv::waitKey(paused ? 0 : 1)};
        // \033 ESC
        if (key == '\033' || key == 'q') {
            return true;
        }
        if (key == ' ') {
            paused = !paused;
        }
    } while (paused);
    return false;
}

#ifdef ENABLE_ROS2

const std::string node_name{"bananas_positioner"};
const std::string image_topic{"aruco_camera/image"};

class RosPositioner : public rclcpp::Node {
  public:
    RosPositioner(world::World &world, visualizer::Visualizer &visualizer,
                  std::shared_ptr<mavsdk::System> mav_system,
                  affine_rotation::AffineRotation camera_to_drone)
        : Node{node_name, rclcpp::NodeOptions{}}, world_{&world},
          visualizer_{&visualizer},
          image_sub_{image_transport::create_subscription(
              this, image_topic,
              [this](const auto &msg) { return imageCallback(msg); }, "raw",
              rmw_qos_profile_sensor_data)},
          camera_to_drone_{std::move(camera_to_drone)} {
        if (mav_system) {
            mocap_.emplace(mav_system);
            fake_mocap_timer_ = create_wall_timer(
                std::chrono::milliseconds{20}, [this] { fakeTimerCallback(); });
            RCLCPP_INFO(this->get_logger(),
                        "Starting to send fake Mocap data.");
        }
    }

  private:
    void
    sendMocap(const mavsdk::Mocap::VisionPositionEstimate &estimate) const {
        Expects(mocap_);
        const auto result{mocap_->set_vision_position_estimate(estimate)};
        if (result != mavsdk::Mocap::Result::Success) {
            std::stringstream ss{};
            ss << result;
            RCLCPP_ERROR(get_logger(), "Failed to send Mocap data: %s",
                         ss.str().c_str());
        }
    }

    void fakeTimerCallback() const {
        mavsdk::Mocap::VisionPositionEstimate estimate{};
        const std::vector<float> covariance_matrix(21);
        estimate.pose_covariance = {covariance_matrix};
        sendMocap(estimate);
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        const cv::Mat image{
            cv_bridge::toCvShare(msg, static_cast<const char *>(
                                          sensor_msgs::image_encodings::BGR8))
                ->image};
        const auto fit_result{world_->fit(image)};

        if (fit_result.camera_to_world && mocap_) {
            if (!fake_mocap_timer_->is_canceled()) {
                RCLCPP_INFO(this->get_logger(),
                            "Found camera position. Stopping fake Mocap data.");
                fake_mocap_timer_->cancel();
            }
            const auto estimate{bananas::mavlink::drone_position_estimate(
                camera_to_drone_, *fit_result.camera_to_world)};
            sendMocap(estimate);
        }

        visualizer_->update(fit_result);
        visualizer_->refresh();

        cv::Mat render_image{};
        image.copyTo(render_image);
        cv::aruco::drawDetectedMarkers(render_image, fit_result.corners,
                                       fit_result.ids);
        cv::imshow("out", render_image);
        if (handle_keys()) {
            rclcpp::shutdown();
        }
    }

    gsl::not_null<world::World *> world_;
    gsl::not_null<visualizer::Visualizer *> visualizer_;
    image_transport::Subscriber image_sub_;
    affine_rotation::AffineRotation camera_to_drone_{};
    std::optional<mavsdk::Mocap> mocap_{};
    rclcpp::TimerBase::SharedPtr fake_mocap_timer_{};
};

#endif // ENABLE_ROS2

} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto main(int argc, char *argv[]) -> int {
#ifdef ENABLE_ROS2
    // Extract the non-ROS arguments.
    const std::vector<std::string> args{
        rclcpp::init_and_remove_ros_arguments(argc, argv)};
    std::vector<const char *> our_argv_vec(args.size());
    std::transform(args.cbegin(), args.cend(), our_argv_vec.begin(),
                   [](const std::string &str) { return str.c_str(); });

    const int our_argc{static_cast<int>(args.size())};
    const char *const *const our_argv{our_argv_vec.data()};
#else  // ENABLE_ROS2
    const int our_argc{argc};
    const char *const *const our_argv{argv};
#endif // ENABLE_ROS2

    cv::CommandLineParser parser{our_argc, our_argv, keys};
    parser.about(about);

    const auto camera_file{parser.get<std::string>("camera")};
    const auto static_environment_file{parser.get<std::string>("env")};
    const auto board_file{parser.get<std::string>("boards")};
#ifndef ENABLE_ROS2
    const auto video_file{parser.get<std::string>(0)};
    std::optional<std::string> video_output_file{};
    if (parser.has("vo")) {
        video_output_file = parser.get<std::string>("vo");
    }
#endif // ENABLE_ROS2

    if (!parser.check()) {
        parser.printErrors();
        parser.printMessage();
        return EXIT_FAILURE;
    }

    cv::Mat camera_matrix{};
    cv::Mat distortion_coefficients{};
    affine_rotation::AffineRotation camera_to_drone{};
    {
        std::ifstream camera_info_stream{camera_file};
        if (!camera_info_stream) {
            std::cerr << "Failed to open camera information file\n";
            return EXIT_FAILURE;
        }

        CameraConfiguration camera_info;
        try {
            const auto json = nlohmann::json::parse(camera_info_stream);
            json.get_to(camera_info);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse camera information file: " << e.what()
                      << '\n';
            return EXIT_FAILURE;
        }
        cv::Mat calibration_matrix{{3, 3},
                                   std::initializer_list<float>{
                                       camera_info.focal_length_x, 0,
                                       camera_info.optical_center_x, 0,
                                       camera_info.focal_length_y,
                                       camera_info.optical_center_y, 0, 0, 1}};
        cv::Mat distortion_mat{camera_info.distortion_coefficients, true};

        camera_matrix = std::move(calibration_matrix);
        distortion_coefficients = std::move(distortion_mat);
        camera_to_drone = camera_info.camera_to_drone;
    }

    world::BoardPlacement static_environment{};
    {
        std::ifstream static_env_stream{static_environment_file};
        if (!static_env_stream) {
            std::cerr << "Failed to open static environment file\n";
            return EXIT_FAILURE;
        }

        try {
            const auto json = nlohmann::json::parse(static_env_stream);
            world::from_json(json, static_environment);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse static environment file: " << e.what()
                      << '\n';
            return EXIT_FAILURE;
        }
    }
    std::vector<board::ConcreteBoard> boards{};
    {
        std::ifstream board_stream{board_file};
        if (!board_stream) {
            std::cerr << "Failed to open board file\n";
            return EXIT_FAILURE;
        }

        try {
            const auto json = nlohmann::json::parse(board_stream);
            json.get_to(boards);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse board file: " << e.what() << '\n';
            return EXIT_FAILURE;
        }
    }

    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{
        mavsdk::ComponentType::CompanionComputer}};
    std::shared_ptr<mavsdk::System> mav_system{};
    if (parser.has("mavlink")) {
        const auto mavlink_url{parser.get<std::string>("mavlink")};
        const auto connection_result{mavsdk.add_any_connection(mavlink_url)};
        if (connection_result != mavsdk::ConnectionResult::Success) {
            std::cerr << "Failed to open MAVLink connection: "
                      << connection_result << '\n';
            return EXIT_FAILURE;
        }
        std::cerr << "Waiting until a MAV system is detected\n";
        while (mavsdk.systems().empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        mav_system = mavsdk.systems().at(0);
        std::cerr << "Found MAV system with id "
                  << static_cast<int>(mav_system->get_system_id()) << '\n';
    }

    const auto dictionary{
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100)};
    world::World world{camera_matrix, distortion_coefficients, dictionary};
    visualizer::Visualizer visualizer{};
    for (const auto &board : boards) {
        std::visit(
            [&world, &visualizer](const auto &settings) {
                const auto board_id{
                    world.addBoard(board::make_board(settings))};
                visualizer.addObject(board_id, settings);
            },
            board);
    }

    for (const auto &[id, placement] : static_environment) {
        world.makeStatic(id, placement);
        visualizer.update(id, placement);
        visualizer.forceVisible(id);
    }

    cv::namedWindow("out", cv::WINDOW_NORMAL);

#ifdef ENABLE_ROS2
    const auto node{std::make_shared<RosPositioner>(
        world, visualizer, mav_system, camera_to_drone)};
    rclcpp::spin(node);
    rclcpp::shutdown();
#else
    std::optional<mavsdk::Mocap> mocap{};
    if (mav_system) {
        mocap.emplace(mav_system);
    }

    cv::VideoCapture capture{video_file};
    cv::VideoWriter output{};
    if (video_output_file) {
        output.open(*video_output_file,
                    cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                    capture.get(cv::CAP_PROP_FPS),
                    {static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
                     static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))});
    }

    cv::Mat image{};
    cv::Mat render_image{};
    const auto start_time{std::chrono::system_clock::now()};
    while (capture.isOpened()) {
        const bool got_frame{capture.read(image)};
        if (!got_frame) {
            break;
        }

        const auto fit_result{world.fit(image)};
        if (fit_result.camera_to_world && mocap) {
            const auto estimate{bananas::mavlink::drone_position_estimate(
                camera_to_drone, *fit_result.camera_to_world)};
            const auto result{mocap->set_vision_position_estimate(estimate)};
            if (result != mavsdk::Mocap::Result::Success) {
                std::cerr << "Failed to send Mocap data: " << result << '\n';
            }
        }

        visualizer.update(fit_result);
        visualizer.refresh();

        image.copyTo(render_image);
        cv::aruco::drawDetectedMarkers(render_image, fit_result.corners,
                                       fit_result.ids);
        if (video_output_file) {
            output.write(render_image);
        } else {
            const std::chrono::nanoseconds target_after_start{
                1'000'000 *
                static_cast<std::int64_t>(capture.get(cv::CAP_PROP_POS_MSEC))};
            std::this_thread::sleep_until(start_time + target_after_start);

            cv::imshow("out", render_image);
            if (handle_keys()) {
                break;
            }
        }
    }
#endif
}
