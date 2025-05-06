#include <bananas_aruco/world.h>

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include <gsl/assert>

#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/board.h>

namespace {

struct PlacementJson {
    std::uint32_t id{};
    affine_rotation::AffineRotation board_to_world{};
};
// Mark to_json as potentially unused to make clang happy. This is ugly, but it
// works.
[[maybe_unused]] NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlacementJson, id,
                                                    board_to_world);

} // namespace

namespace world {

void from_json(const nlohmann::json &j, BoardPlacement &placement) {
    const auto placement_vector{j.get<std::vector<PlacementJson>>()};

    placement = {};
    placement.reserve(placement_vector.size());
    for (const auto &board : placement_vector) {
        placement.emplace(board.id, board.board_to_world);
    }
}

World::World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
             const cv::aruco::Dictionary &dictionary)
    : camera_matrix_{std::move(camera_matrix)},
      distortion_coeffs_{std::move(distortion_coeffs)},
      dictionary_{&dictionary}, detector_{dictionary, {}},
      static_environment_{cv::Mat(0, 0, CV_32FC3), dictionary, {}} {}

auto World::addBoard(const board::Board &board) -> BoardId {
    all_boards_.push_back(board::to_cv(*dictionary_, board));
    return static_cast<std::uint32_t>(all_boards_.size()) - 1;
}

void World::makeStatic(BoardId id,
                       const affine_rotation::AffineRotation &board_to_world) {
    Expects(id < all_boards_.size());
    static_board_placements_.emplace(id, board_to_world);
    // TODO(vainiovano): Avoid regenerating the environment from scratch every
    // time?
    recomputeStaticEnvironment();
}

auto World::fit(const cv::Mat &image) const -> FitResult {
    std::vector<std::vector<cv::Point2f>> corners{};
    std::vector<std::vector<cv::Point2f>> rejected{};
    std::vector<int> ids{};
    detector_.detectMarkers(image, corners, ids, rejected);
    detector_.refineDetectedMarkers(image, static_environment_, corners, ids,
                                    rejected, camera_matrix_,
                                    distortion_coeffs_);
    for (BoardId board_id{0}; board_id < all_boards_.size(); ++board_id) {
        if (static_board_placements_.find(board_id) !=
            static_board_placements_.cend()) {
            continue;
        }
        detector_.refineDetectedMarkers(image, all_boards_[board_id], corners,
                                        ids, rejected, camera_matrix_,
                                        distortion_coeffs_);
    }

    std::optional<UncertainPose> camera_to_world{};
    UncertainPlacement dynamic_board_placements{};
    const auto static_environment_fit{
        fitBoard(corners, ids, static_environment_)};
    if (static_environment_fit) {
        camera_to_world = {static_environment_fit->reprojection_error,
                           static_environment_fit->placement.inverse()};
        // Help clang-tidy 18's bugprone-unchecked-optional-access lint. Newer
        // versions are smarter.
        const auto &camera_to_world_placement{camera_to_world->placement};
        // TODO(vainiovano): Allow producing results even if the exact camera
        // location is not known.
        for (BoardId board_id{0}; board_id < all_boards_.size(); ++board_id) {
            if (static_board_placements_.find(board_id) !=
                static_board_placements_.cend()) {
                continue;
            }
            const auto board_to_camera{
                fitBoard(corners, ids, all_boards_[board_id])};
            if (board_to_camera) {
                // TODO(vainiovano): Combine the reprojection error with that of
                // the camera location?
                dynamic_board_placements.emplace(
                    board_id, UncertainPose{board_to_camera->reprojection_error,
                                            camera_to_world_placement *
                                                board_to_camera->placement});
            }
        }
    }

    return {std::move(corners), std::move(ids), std::move(camera_to_world),
            std::move(dynamic_board_placements)};
}

auto World::fitBoard(const std::vector<std::vector<cv::Point2f>> &corners,
                     const std::vector<int> &ids, const cv::aruco::Board &board)
    const -> std::optional<UncertainPose> {
    if (ids.empty()) {
        return {};
    }

    cv::Mat object_points;
    cv::Mat image_points;
    board.matchImagePoints(corners, ids, object_points, image_points);
    if (object_points.rows / 4 < min_marker_count) {
        return {};
    }

    std::vector<cv::Vec3f> rvecs;
    std::vector<cv::Vec3f> tvecs;
    std::vector<float> reprojection_errors;
    const auto num_solutions{cv::solvePnPGeneric(
        object_points, image_points, camera_matrix_, distortion_coeffs_, rvecs,
        tvecs, false, cv::SOLVEPNP_ITERATIVE, cv::noArray(), cv::noArray(),
        reprojection_errors)};
    if (num_solutions == 0) {
        return {};
    }

    const auto opencv_placement{affine_rotation::from_cv(rvecs[0], tvecs[0])};
    // The OpenCV camera coordinate system has X right and Y up while we want
    // them the opposite way around as in the glTF coordinate system. Rotate the
    // coordinates by 180° around the Z axis.
    const auto gltf_placement{affine_rotation::AffineRotation{
                                  Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf{0.0F, 0.0F, 0.0F, 1.0F}} *
                              opencv_placement};
    return {{reprojection_errors[0], gltf_placement}};
}

void World::recomputeStaticEnvironment() {
    std::vector<std::vector<cv::Point3f>> obj_points{};
    std::vector<int> ids{};

    for (const auto &[id, placement] : static_board_placements_) {
        const auto board{all_boards_[id]};
        const auto &board_to_world{placement};

        obj_points.reserve(obj_points.size() + board.getObjPoints().size());
        std::transform(
            board.getObjPoints().cbegin(), board.getObjPoints().cend(),
            std::back_inserter(obj_points),
            [&board_to_world](const std::vector<cv::Point3f> &points) {
                std::vector<cv::Point3f> res(4);
                std::transform(points.cbegin(), points.cend(), res.begin(),
                               [&board_to_world](cv::Point3f point) {
                                   return board_to_world * point;
                               });
                return res;
            });

        ids.insert(ids.end(), board.getIds().cbegin(), board.getIds().cend());
    }
    static_environment_ = {obj_points, *dictionary_, ids};
}

} // namespace world
