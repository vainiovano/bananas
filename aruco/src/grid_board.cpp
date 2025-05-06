#include <bananas_aruco/grid_board.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <opencv2/core/types.hpp>

#include <bananas_aruco/board.h>

namespace board {

auto grid_width(const GridSettings &settings) -> float {
    return (static_cast<float>(settings.size.num_columns) *
            settings.marker_side) +
           (static_cast<float>(settings.size.num_columns - 1) *
            settings.marker_separation);
}

auto grid_height(const GridSettings &settings) -> float {
    return (static_cast<float>(settings.size.num_rows) * settings.marker_side) +
           (static_cast<float>(settings.size.num_rows - 1) *
            settings.marker_separation);
}

auto make_board(const GridSettings &settings) -> Board {
    std::vector<std::vector<cv::Point3f>> object_points{};
    std::vector<int> ids{};

    object_points.reserve(std::size_t{settings.size.num_columns} *
                          settings.size.num_rows);
    ids.reserve(std::size_t{settings.size.num_columns} *
                settings.size.num_rows);

    const float plane_width{grid_width(settings)};
    const float plane_height{grid_height(settings)};
    const float plane_left{-plane_width / 2.0F};
    const float plane_top{-plane_height / 2.0F};

    int id{settings.start_id};
    for (std::uint32_t y{0}; y < settings.size.num_rows; ++y) {
        const float marker_top{
            plane_top + (static_cast<float>(y) *
                         (settings.marker_side + settings.marker_separation))};
        for (std::uint32_t x{0}; x < settings.size.num_columns; ++x) {
            const float marker_left{
                plane_left +
                (static_cast<float>(x) *
                 (settings.marker_side + settings.marker_separation))};

            ids.push_back(id++);
            object_points.push_back(
                {cv::Point3f{marker_left, 0.0F, marker_top},
                 cv::Point3f{marker_left + settings.marker_side, 0.0F,
                             marker_top},
                 cv::Point3f{marker_left + settings.marker_side, 0.0F,
                             marker_top + settings.marker_side},
                 cv::Point3f{marker_left, 0.0F,
                             marker_top + settings.marker_side}});
        }
    }

    return {std::move(object_points), std::move(ids)};
}

} // namespace board
