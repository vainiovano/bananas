#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/types.hpp>

#include <bananas_aruco/grid_board.h>

TEST(GridBoardTest, GridBoardsWork) {
    const board::GridSettings settings{{3, 2}, 1.0F, 0.5F, 5};
    auto board{board::make_board(settings)};

    const std::vector<int> expected_ids{5, 6, 7, 8, 9, 10};
    EXPECT_EQ(board.marker_ids, expected_ids);

    ASSERT_EQ(board.obj_points.size(), 6);

    const std::vector<cv::Point3f> expected_top_left{
        {-2.0F, 0.0F, -1.25F},
        {-1.0F, 0.0F, -1.25F},
        {-1.0F, 0.0F, -0.25F},
        {-2.0F, 0.0F, -0.25F},
    };
    EXPECT_EQ(board.obj_points[0], expected_top_left);

    const std::vector<cv::Point3f> expected_top_middle{
        {-0.5F, 0.0F, -1.25F},
        {0.5F, 0.0F, -1.25F},
        {0.5F, 0.0F, -0.25F},
        {-0.5F, 0.0F, -0.25F},
    };
    EXPECT_EQ(board.obj_points[1], expected_top_middle);

    const std::vector<cv::Point3f> expected_top_right{
        {1.0F, 0.0F, -1.25F},
        {2.0F, 0.0F, -1.25F},
        {2.0F, 0.0F, -0.25F},
        {1.0F, 0.0F, -0.25F},
    };
    EXPECT_EQ(board.obj_points[2], expected_top_right);

    const std::vector<cv::Point3f> expected_bottom_left{
        {-2.0F, 0.0F, 0.25F},
        {-1.0F, 0.0F, 0.25F},
        {-1.0F, 0.0F, 1.25F},
        {-2.0F, 0.0F, 1.25F},
    };
    EXPECT_EQ(board.obj_points[3], expected_bottom_left);

    const std::vector<cv::Point3f> expected_bottom_middle{
        {-0.5F, 0.0F, 0.25F},
        {0.5F, 0.0F, 0.25F},
        {0.5F, 0.0F, 1.25F},
        {-0.5F, 0.0F, 1.25F},
    };
    EXPECT_EQ(board.obj_points[4], expected_bottom_middle);

    const std::vector<cv::Point3f> expected_bottom_right{
        {1.0F, 0.0F, 0.25F},
        {2.0F, 0.0F, 0.25F},
        {2.0F, 0.0F, 1.25F},
        {1.0F, 0.0F, 1.25F},
    };
    EXPECT_EQ(board.obj_points[5], expected_bottom_right);
}
