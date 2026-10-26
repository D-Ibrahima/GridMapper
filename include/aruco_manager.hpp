#pragma once
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// OpenCV >= 4.7 : nouvelle API (ArucoDetector). Avant : ancienne API (contrib).
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    #define ARUCO_NEW_API 1
    using ArucoDictType = cv::aruco::PredefinedDictionaryType;
#else
    #define ARUCO_NEW_API 0
    using ArucoDictType = cv::aruco::PREDEFINED_DICTIONARY_NAME;
#endif

class ArucoManager {
public:
    ArucoManager(ArucoDictType dictType = cv::aruco::DICT_6X6_250);
    ~ArucoManager() = default;

    // Retourne true UNIQUEMENT si au moins un marqueur est détecté
    // (id_detected n'est modifié que dans ce cas).
    bool detectAruco(int &id_detected, cv::Mat &frame);

private:
    ArucoDictType dictType_;
#if ARUCO_NEW_API
    cv::aruco::ArucoDetector detector_;
#else
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
#endif
};
