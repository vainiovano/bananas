// This file is a modified version of the file
// modules/objdetect/src/aruco/aruco_utils.hpp from the OpenCV project.
// It is subject to the license terms in the LICENSE file found in the
// aruco_detector directory.
#ifndef __BANANAS_ARUCO_UTILS_HPP__
#define __BANANAS_ARUCO_UTILS_HPP__

#include <opencv2/core.hpp>
#include <vector>

namespace aruco_detector {

using namespace cv;

/**
 * @brief Copy the contents of a corners vector to an OutputArray, settings its size.
 */
void _copyVector2Output(std::vector<std::vector<Point2f> > &vec, OutputArrayOfArrays out, const float scale = 1.f);

/**
  * @brief Convert input image to gray if it is a BGR or BGRA image
  */
void _convertToGrey(InputArray _in, Mat& _out);

template<typename T>
inline bool readParameter(const std::string& name, T& parameter, const FileNode& node)
{
    if (!node.empty() && !node[name].empty()) {
        node[name] >> parameter;
        return true;
    }
    return false;
}

template<typename T>
inline bool readWriteParameter(const std::string& name, T& parameter, const FileNode* readNode, FileStorage* writeStorage)
{
    if (readNode)
        return readParameter(name, parameter, *readNode);
    CV_Assert(writeStorage);
    *writeStorage << name << parameter;
    return true;
}

}
#endif
