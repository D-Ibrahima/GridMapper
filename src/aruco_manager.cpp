#include <vector>
#include "aruco_manager.hpp"

#if ARUCO_NEW_API
ArucoManager::ArucoManager(ArucoDictType dictType)
    : dictType_(dictType),
      detector_(cv::aruco::getPredefinedDictionary(dictType), cv::aruco::DetectorParameters()) {}
#else
ArucoManager::ArucoManager(ArucoDictType dictType)
    : dictType_(dictType),
      dict_(cv::aruco::getPredefinedDictionary(dictType)),
      params_(cv::aruco::DetectorParameters::create()) {}
#endif

bool ArucoManager::detectAruco(int &id_detected, cv::Mat &frame) {
    if (frame.empty()) return false;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

#if ARUCO_NEW_API
    detector_.detectMarkers(frame, corners, ids);
#else
    cv::aruco::detectMarkers(frame, dict_, corners, ids, params_);
#endif

    if (ids.empty()) return false;

    cv::aruco::drawDetectedMarkers(frame, corners, ids, cv::Scalar(255, 0, 0));
    id_detected = ids.front();
    return true;
}
