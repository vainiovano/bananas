#ifndef AFFINE_ROTATION_H_
#define AFFINE_ROTATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nlohmann/json_fwd.hpp>

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

namespace affine_rotation {

/// A combination of translation and rotation.
class AffineRotation {
  public:
    /// An identity transform.
    AffineRotation();

    /// A transform with the given translation and rotation.
    AffineRotation(Eigen::Vector3f translation, Eigen::Quaternionf rotation);

    auto operator*(const AffineRotation &other) const -> AffineRotation;
    void operator*=(const AffineRotation &other);

    auto operator*(cv::Point3f point) const -> cv::Point3f;

    [[nodiscard]] auto inverse() const -> AffineRotation;

    [[nodiscard]] auto getTranslation() const -> Eigen::Vector3f;
    [[nodiscard]] auto getRotation() const -> Eigen::Quaternionf;

  private:
    Eigen::Vector3f translation_;
    Eigen::Quaternionf rotation_;
};

void from_json(const nlohmann::json &j, AffineRotation &affine_rotation);
void to_json(nlohmann::json &j, const AffineRotation &affine_rotation);

auto from_cv(const cv::Vec3f &rvec, const cv::Vec3f &tvec) -> AffineRotation;

} // namespace affine_rotation

#endif // AFFINE_ROTATION_H_
