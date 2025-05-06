#include <string>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/mavlink.h>

namespace {

constexpr float pi{3.1415927F};
// EXPECT_FLOAT_EQ's usage of ULPs gets a bit silly near zero. Let's just use a
// constant absolute bound instead.
constexpr float error_bound{1E-6};

} // namespace

TEST(BananasMavlinkTest, DronePositionEstimateWorks) {
    const affine_rotation::AffineRotation camera_to_drone{
        Eigen::Vector3f{0.0F, -0.125F, 0.0F},
        Eigen::Quaternionf{
            Eigen::AngleAxisf{pi / 2.0F, Eigen::Vector3f::UnitX()}}};

    for (int x_i{-1}; x_i <= 1; ++x_i) {
        for (int z_i{-1}; z_i <= 1; ++z_i) {
            for (int yaw_i{0}; yaw_i <= 4; ++yaw_i) {
                const float x{static_cast<float>(x_i)};
                const float z{static_cast<float>(z_i)};
                const float yaw{static_cast<float>(yaw_i) * (pi / 2.0F)};
                SCOPED_TRACE("x = " + std::to_string(x) +
                             ", z = " + std::to_string(z) +
                             ", yaw = " + std::to_string(yaw));

                const affine_rotation::AffineRotation camera_to_world{
                    Eigen::Vector3f{x, 0.5F, z},
                    Eigen::Quaternionf{
                        Eigen::AngleAxisf{yaw, Eigen::Vector3f::UnitY()} *
                        Eigen::AngleAxisf{pi / 2.0F,
                                          Eigen::Vector3f::UnitX()}}};

                const auto position_estimate{
                    bananas::mavlink::drone_position_estimate(
                        camera_to_drone, {0.0F, camera_to_world})};
                EXPECT_NEAR(position_estimate.position_body.x_m, z,
                            error_bound);
                EXPECT_NEAR(position_estimate.position_body.y_m, -x,
                            error_bound);
                EXPECT_NEAR(position_estimate.position_body.z_m, -0.625F,
                            error_bound);

                const Eigen::Quaternionf angle_body{
                    Eigen::AngleAxisf{position_estimate.angle_body.yaw_rad,
                                      Eigen::Vector3f::UnitZ()} *
                    Eigen::AngleAxisf{position_estimate.angle_body.pitch_rad,
                                      Eigen::Vector3f::UnitY()} *
                    Eigen::AngleAxisf{position_estimate.angle_body.roll_rad,
                                      Eigen::Vector3f::UnitX()}};
                const Eigen::Quaternionf expected_angle_body{
                    Eigen::AngleAxisf{yaw, -Eigen::Vector3f::UnitZ()}};
                EXPECT_NEAR(angle_body.angularDistance(expected_angle_body),
                            0.0F, error_bound);
            }
        }
    }
}
