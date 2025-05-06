#include <bananas_aruco/mavlink.h>

#include <limits>

#include <mavsdk/plugins/mocap/mocap.h>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/world.h>

namespace bananas::mavlink {

mavsdk::Mocap::VisionPositionEstimate
drone_position_estimate(const affine_rotation::AffineRotation &camera_to_drone,
                        const world::UncertainPose &camera_to_world) {
    mavsdk::Mocap::VisionPositionEstimate estimate{};

    Eigen::Matrix3f gltf_to_ned{};
    // clang-format off
    gltf_to_ned <<
        0.0F,  0.0F,  1.0F,
       -1.0F,  0.0F,  0.0F,
        0.0F, -1.0F,  0.0F;
    // clang-format on

    const auto drone_to_world{camera_to_world.placement *
                              camera_to_drone.inverse()};
    const Eigen::Vector3f translation{gltf_to_ned *
                                      drone_to_world.getTranslation()};
    estimate.position_body = {translation.x(), translation.y(),
                              translation.z()};

    const Eigen::Matrix3f rotation{
        gltf_to_ned * drone_to_world.getRotation().toRotationMatrix() *
        gltf_to_ned.inverse()};
    const Eigen::Vector3f euler_angles{rotation.eulerAngles(2, 1, 0)};
    estimate.angle_body = {euler_angles.z(), euler_angles.y(),
                           euler_angles.x()};
    estimate.pose_covariance = {{std::numeric_limits<float>::quiet_NaN()}};

    return estimate;
}

} // namespace bananas::mavlink
