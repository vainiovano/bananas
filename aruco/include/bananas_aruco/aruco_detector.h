#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#ifdef ENABLE_CUSTOM_DETECTOR

#include <bananas_aruco_detector/aruco_detector.hpp>

#else // ENABLE_CUSTOM_DETECTOR

#include <opencv2/objdetect/aruco_detector.hpp>

namespace aruco_detector {

using ArucoDetector = cv::aruco::ArucoDetector;
using DetectorParameters = cv::aruco::DetectorParameters;

} // namespace aruco_detector

#endif // ENABLE_CUSTOM_DETECTOR

#endif // ARUCO_DETECTOR_H_
